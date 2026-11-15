"""上机自检：依赖、串口、相机、ArUco、机械臂。

用法：
  python hardware_check.py                 # 按 config.yaml 检查
  python hardware_check.py --skip-arm      # 只测视觉
"""

import argparse
import importlib.util
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_cfg(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def has(mod):
    return importlib.util.find_spec(mod) is not None


def check_ports(cfg_port):
    if not has("serial"):
        return "SKIP", "pyserial 未安装（随 lerobot 一起装）"
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    names = [p.device for p in ports]
    if not ports:
        return "FAIL", "没有串口设备（机械臂没插/驱动没装）"
    if cfg_port in names:
        return "PASS", f"找到配置端口 {cfg_port}"
    return "WARN", f"有串口 {names}，但配置里是 {cfg_port}"


def check_camera(cfg):
    cap = cv2.VideoCapture(cfg["camera"]["index"])
    if not cap.isOpened():
        return "FAIL", "相机打不开", None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["camera"]["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["camera"]["height"])
    cap.set(cv2.CAP_PROP_FPS, cfg["camera"]["fps"])

    t0 = time.monotonic()
    n = 0
    frame = None
    while n < 30 and time.monotonic() - t0 < 3:
        ok, f = cap.read()
        if ok:
            frame = f
            n += 1
    dt = time.monotonic() - t0
    cap.release()
    if frame is None:
        return "FAIL", "读不到画面", None
    fps = n / max(dt, 1e-3)
    h, w = frame.shape[:2]
    return "PASS", f"{w}x{h} @ {fps:.0f}fps", frame


def check_marker(cfg, frame, base):
    intr = base / cfg["camera"]["intrinsics"]
    if not intr.exists():
        return "SKIP", f"缺 {intr}，先跑 calib.py intrinsics"
    try:
        from tracker import MarkerTracker
        tr = MarkerTracker(cfg, base)
        T, corners = tr.detect(frame)
    except Exception as e:
        return "FAIL", f"检测异常: {e}"
    if cv2.aruco is None:
        return "FAIL", "opencv 没有 aruco 模块"
    if T is None:
        return "SKIP", "画面里没有标记（把 40mm 标记放到相机前）"
    t = T[:3, 3]
    return "PASS", f"id={cfg['marker']['id']} 位置 {t[0]:.0f},{t[1]:.0f},{t[2]:.0f} mm"


def check_arm(cfg, base):
    if not has("lerobot"):
        return "SKIP", "lerobot 未安装"
    urdf = base / cfg["arm"]["urdf"]
    if not urdf.exists():
        return "SKIP", f"缺 URDF: {urdf}"
    try:
        from arm import make_arm
        arm = make_arm(cfg["arm"])
        q = arm.read()
        arm.release()
        return "PASS", "关节: " + ", ".join(f"{v:.1f}" for v in q)
    except Exception as e:
        return "FAIL", f"连接/读取失败: {e}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--skip-arm", action="store_true")
    args = ap.parse_args()

    path = Path(args.config)
    cfg = load_cfg(path)
    base = path.parent

    rows = []
    for mod, hint in [("numpy", ""), ("cv2", ""), ("yaml", ""), ("serial", "pip install pyserial"),
                      ("lerobot", 'pip install "lerobot[placo-dep]"'), ("placo", "随 lerobot 安装")]:
        rows.append((f"依赖 {mod}", "PASS" if has(mod) else "SKIP", hint or "ok"))

    status, msg = check_ports(cfg["arm"]["port"])
    rows.append(("串口", status, msg))

    status, msg, frame = check_camera(cfg)
    rows.append(("相机", status, msg))

    if frame is not None:
        status, msg = check_marker(cfg, frame, base)
        rows.append(("ArUco", status, msg))

    if not args.skip_arm:
        status, msg = check_arm(cfg, base)
        rows.append(("机械臂", status, msg))

    print()
    width = max(len(r[0]) for r in rows) + 2
    for name, status, msg in rows:
        print(f"{name:<{width}} {status:<5} {msg}")

    fails = [r for r in rows if r[1] == "FAIL"]
    skips = [r for r in rows if r[1] == "SKIP"]
    if fails:
        print(f"\n结论: {len(fails)} 项失败，先处理 FAIL")
    elif skips:
        print(f"\n结论: 没有失败，但 {len(skips)} 项 SKIP（硬件/依赖未到位）")
    else:
        print("\n结论: 全部通过")


if __name__ == "__main__":
    main()
