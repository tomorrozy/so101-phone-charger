"""标定流程：内参 -> 采数据 -> 手眼 -> 贴合示教。

用法（全部在 desk-charger 目录下执行）：
  python calib.py intrinsics --images calib_data/chess
  python calib.py capture --out calib_data/handeye
  python calib.py handeye --data calib_data/handeye
  python calib.py dock
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from arm import make_arm
from tracker import MarkerTracker, inv_T, make_T


def solve_handeye(T_base_ee, T_cam_marker):
    """求解 T_base_ee @ X @ T_cam_marker 恒定的 X（Park 法）。

    OpenCV 5 的 Python 绑定里没有 calibrateHandEye，这里用 SVD 自己解 AX=XB。
    """
    n = len(T_base_ee)
    M = np.zeros((3, 3))
    pairs = []
    for i in range(n):
        for j in range(i + 1, n):
            A = inv_T(T_base_ee[j]) @ T_base_ee[i]
            B = T_cam_marker[j] @ inv_T(T_cam_marker[i])
            alpha = cv2.Rodrigues(A[:3, :3])[0].ravel()
            beta = cv2.Rodrigues(B[:3, :3])[0].ravel()
            M += np.outer(beta, alpha)
            pairs.append((A, B))

    U, _, Vt = np.linalg.svd(M)
    R = Vt.T @ np.diag([1.0, 1.0, np.linalg.det(Vt.T @ U.T)]) @ U.T

    C, d = [], []
    for A, B in pairs:
        C.append(A[:3, :3] - np.eye(3))
        d.append(R @ B[:3, 3] - A[:3, 3])
    t = np.linalg.lstsq(np.vstack(C), np.hstack(d), rcond=None)[0]
    return make_T(R, t)


def load_cfg(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def cmd_intrinsics(args):
    cols, rows, square = args.cols, args.rows, args.square
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square

    obj_points, img_points = [], []
    files = sorted(Path(args.images).glob("*.jpg")) + sorted(Path(args.images).glob("*.png"))
    for f in files:
        img = cv2.imread(str(f))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCornersSB(
            gray, (cols, rows), flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        )
        if not ok:
            print(f"跳过（找不到棋盘）: {f.name}")
            continue
        obj_points.append(obj)
        img_points.append(corners.astype(np.float32))
        print(f"ok {f.name}")

    if len(obj_points) < 8:
        raise SystemExit("有效图片不足 8 张，补拍")

    h, w = cv2.imread(str(files[0])).shape[:2]
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            {
                "camera_matrix": K.ravel().tolist(),
                "dist_coeffs": dist.ravel().tolist(),
                "image_size": [w, h],
                "rms": float(rms),
            },
            f,
        )
    print(f"内参写入 {out}，重投影误差 {rms:.3f}px")


def cmd_capture(args):
    cfg = load_cfg(args.config)
    base = Path(args.config).parent
    arm = make_arm(cfg["arm"])
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(cfg["camera"]["index"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["camera"]["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["camera"]["height"])

    print("把 40mm ArUco 平放在桌面不动，摆 15–20 个不同姿态。回车采样，q 结束。")
    n = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise SystemExit("相机打不开")
            cv2.imshow("capture", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key in (13, 10):
                q = arm.read()
                T = arm.fk(q)
                idx = f"{n:03d}"
                cv2.imwrite(str(out / f"{idx}.jpg"), frame)
                with open(out / f"{idx}.json", "w", encoding="utf-8") as f:
                    json.dump({"joints_deg": q.tolist(), "T_base_ee": T.tolist()}, f)
                print(f"采样 {idx}")
                n += 1
    finally:
        cap.release()
        cv2.destroyAllWindows()
        arm.release()
    print(f"共 {n} 张，下一步: python calib.py handeye --data {out}")


def _pairs(data_dir, cfg):
    """返回 T_base_ee 列表和 T_cam_marker 列表。"""
    base = Path(data_dir)
    tracker = MarkerTracker(cfg, Path(cfg["_config_dir"]))
    g2b, t2c = [], []
    for js in sorted(base.glob("*.json")):
        img = cv2.imread(str(base / (js.stem + ".jpg")))
        T_cam_marker, _ = tracker.detect(img)
        if T_cam_marker is None:
            print(f"跳过（没有标记）: {js.name}")
            continue
        with open(js, "r", encoding="utf-8") as f:
            T_base_ee = np.array(json.load(f)["T_base_ee"], dtype=float)
        g2b.append(T_base_ee)
        t2c.append(T_cam_marker)
    return g2b, t2c


def cmd_handeye(args):
    cfg = load_cfg(args.config)
    cfg["_config_dir"] = str(Path(args.config).parent)
    g2b, t2c = _pairs(args.data, cfg)
    if len(g2b) < 8:
        raise SystemExit(f"有效样本 {len(g2b)} 组，至少 8 组，建议 15 组以上")

    T_ee_cam = solve_handeye(g2b, t2c)

    pts = np.array([(T @ T_ee_cam @ Tcm)[:3, 3] for T, Tcm in zip(g2b, t2c)])
    spread = pts.std(axis=0)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        yaml.safe_dump({"T_ee_cam": T_ee_cam.ravel().tolist(), "spread_mm": spread.tolist()}, f)
    print(f"手眼写入 {out}")
    print(f"标记在基座系的位置散布: {spread.round(2)} mm（应 < 3mm）")


def cmd_deliver(args):
    cfg = load_cfg(args.config)
    base_dir = Path(args.config).parent
    arm = make_arm(cfg["arm"])
    print("把手机吸在盘上，把臂摆到想送达的位置，回车保存，q 取消。")
    try:
        ch = input()
        if ch.strip().lower() == "q":
            raise SystemExit("取消")
        T = arm.fk(arm.read())
        out = base_dir / "calib" / "deliver.yaml"
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            yaml.safe_dump({"T_base_ee": T.ravel().tolist()}, f)
        print(f"送达位姿写入 {out}")
    finally:
        arm.release()


def cmd_dock(args):
    cfg = load_cfg(args.config)
    cfg["_config_dir"] = str(Path(args.config).parent)
    base = Path(args.config).parent
    arm = make_arm(cfg["arm"])
    tracker = MarkerTracker(cfg, base)
    cap = cv2.VideoCapture(cfg["camera"]["index"])

    print("把盘贴在手机背面标记中心，保持不动，回车采样。")
    try:
        while True:
            ok, frame = cap.read()
            cv2.imshow("dock", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                raise SystemExit("取消")
            if key in (13, 10):
                T_base_ee = arm.fk(arm.read())
                target, _ = tracker.update(frame, T_base_ee)
                if not target.visible:
                    print("看不到标记，换个角度")
                    continue
                d0 = float(np.dot(T_base_ee[:3, 3] - target.pos_mm, target.normal))
                out = base / cfg["dock"]
                out.parent.mkdir(parents=True, exist_ok=True)
                with open(out, "w", encoding="utf-8") as f:
                    yaml.safe_dump(
                        {
                            "T_base_ee0": T_base_ee.ravel().tolist(),
                            "normal0": target.normal.tolist(),
                            "d0_mm": d0,
                        },
                        f,
                    )
                print(f"示教写入 {out}，d0={d0:.1f}mm")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        arm.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("intrinsics")
    p.add_argument("--images", default="calib_data/chess")
    p.add_argument("--cols", type=int, default=9)
    p.add_argument("--rows", type=int, default=6)
    p.add_argument("--square", type=float, default=25.0)
    p.add_argument("--out", default="calib/intrinsics.yaml")
    p.set_defaults(func=cmd_intrinsics)

    p = sub.add_parser("capture")
    p.add_argument("--out", default="calib_data/handeye")
    p.set_defaults(func=cmd_capture)

    p = sub.add_parser("handeye")
    p.add_argument("--data", default="calib_data/handeye")
    p.add_argument("--out", default="calib/handeye.yaml")
    p.set_defaults(func=cmd_handeye)

    p = sub.add_parser("dock")
    p.set_defaults(func=cmd_dock)

    p = sub.add_parser("deliver")
    p.set_defaults(func=cmd_deliver)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
