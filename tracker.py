"""ArUco 手机跟踪：图像 -> 基座坐标系的位姿 + 速度估计。"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass
class Target:
    visible: bool
    pos_mm: np.ndarray      # 标记中心（= 线圈中心）在基座系
    normal: np.ndarray      # 单位向量，指向手机背面外侧
    speed_mm_s: float
    stamp: float


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inv_T(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rpy_deg_to_R(rpy):
    r, p, y = np.deg2rad(rpy)
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_T(path, key="T"):
    data = load_yaml(path)[key]
    return np.array(data, dtype=float).reshape(4, 4)


class AlphaBeta:
    """对三维位置做 α-β 滤波，同时给出速度。"""

    def __init__(self, alpha=0.55, beta=0.12):
        self.alpha = alpha
        self.beta = beta
        self.x = None
        self.v = None

    def reset(self, z):
        self.x = z.copy()
        self.v = np.zeros(3)

    def update(self, z, dt):
        if self.x is None:
            self.reset(z)
            return self.x.copy(), self.v.copy()
        dt = max(dt, 1e-3)
        pred = self.x + self.v * dt
        r = z - pred
        self.x = pred + self.alpha * r
        self.v = self.v + (self.beta / dt) * r
        return self.x.copy(), self.v.copy()


class MarkerTracker:
    def __init__(self, cfg, config_dir):
        self.cfg = cfg
        base = Path(config_dir)
        self.marker_id = cfg["marker"]["id"]
        self.marker_mm = cfg["marker"]["size_mm"]

        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, cfg["marker"]["dict"])
        )
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

        intr = load_yaml(base / cfg["camera"]["intrinsics"])
        self.K = np.array(intr["camera_matrix"], dtype=float).reshape(3, 3)
        self.dist = np.array(intr["dist_coeffs"], dtype=float)

        handeye = base / cfg["handeye"]
        if handeye.exists():
            self.T_ee_cam = load_T(handeye, "T_ee_cam")
        else:
            print(f"警告：{handeye} 不存在，暂时按单位阵处理（只影响跟踪，不影响检测）")
            self.T_ee_cam = np.eye(4)

        s = self.marker_mm / 2.0
        self.obj = np.array(
            [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]],
            dtype=np.float32,
        )
        self.filt = AlphaBeta()
        self.normal = None
        self.stamp = 0.0
        self.latency = cfg["control"]["latency_s"]

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None, None
        ids = ids.ravel()
        hits = np.where(ids == self.marker_id)[0]
        if hits.size == 0:
            return None, corners
        pts = corners[hits[0]].reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            self.obj, pts, self.K, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            return None, corners
        R, _ = cv2.Rodrigues(rvec)
        return make_T(R, tvec.ravel()), corners

    def update(self, frame, T_base_ee):
        T_cam_marker, corners = self.detect(frame)
        now = cv2.getTickCount() / cv2.getTickFrequency()
        if T_cam_marker is None:
            if self.stamp and now - self.stamp > 0.5:
                self.filt = AlphaBeta()
            return Target(False, np.zeros(3), np.zeros(3), 0.0, now), corners

        T_base_marker = T_base_ee @ self.T_ee_cam @ T_cam_marker
        pos = T_base_marker[:3, 3]
        n = T_base_marker[:3, 2]
        n /= np.linalg.norm(n)

        dt = now - self.stamp if self.stamp else 1.0 / self.cfg["control"]["hz"]
        self.stamp = now
        pos_f, vel = self.filt.update(pos, dt)
        pos_f = pos_f + vel * self.latency

        self.normal = n if self.normal is None else 0.7 * self.normal + 0.3 * n
        self.normal /= np.linalg.norm(self.normal)

        return (
            Target(True, pos_f, self.normal.copy(), float(np.linalg.norm(vel)), now),
            corners,
        )
