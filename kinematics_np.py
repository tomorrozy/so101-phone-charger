"""纯 numpy 的 URDF 正/逆解，给没有 placo 的平台（Windows）用。

坐标统一用 mm：URDF 的 origin 在解析时乘 scale（默认 1000）。
接口和 lerobot.model.kinematics.RobotKinematics 对齐：
  forward_kinematics(deg) -> 4x4 (mm)
  inverse_kinematics(current_deg, desired_4x4, position_weight, orientation_weight, max_iters) -> deg
"""

import math
import xml.etree.ElementTree as ET

import numpy as np


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_R(axis, q):
    a = np.array(axis, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-9:
        return np.eye(3)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def _rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-7:
        return np.zeros(3)
    if abs(ang - math.pi) < 1e-4:
        w, v = np.linalg.eigh((R + np.eye(3)) / 2.0)
        axis = v[:, np.argmax(w)]
        return axis * ang
    return ang / (2.0 * math.sin(ang)) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]
    )


class SimpleKinematics:
    def __init__(self, urdf_path, joint_names, target_frame="gripper_frame_link", scale=1000.0):
        self.names = list(joint_names)
        self.target = target_frame
        self.scale = float(scale)
        joints = {}
        for j in ET.parse(urdf_path).getroot().findall("joint"):
            o = j.find("origin")
            ax = j.find("axis")
            lim = j.find("limit")
            child = j.find("child").get("link")
            joints[child] = {
                "name": j.get("name"),
                "type": j.get("type"),
                "xyz": np.array([float(v) for v in (o.get("xyz").split() if o is not None else "0 0 0")]) * self.scale,
                "rpy": [float(v) for v in (o.get("rpy").split() if o is not None else "0 0 0")],
                "axis": [float(v) for v in (ax.get("xyz").split() if ax is not None else "0 0 1")],
                "lo": float(lim.get("lower")) if lim is not None else -math.pi,
                "hi": float(lim.get("upper")) if lim is not None else math.pi,
                "parent": j.find("parent").get("link"),
                "child": child,
            }
        chain = []
        link = self.target
        while link in joints:
            j = joints[link]
            chain.append(j)
            link = j["parent"]
        if link != "base_link":
            raise ValueError(f"URDF 里找不到 base_link -> {self.target} 的链路")
        chain.reverse()
        self.chain = chain
        self.index = {n: i for i, n in enumerate(self.names)}

    def _fk(self, q_deg):
        T = np.eye(4)
        for j in self.chain:
            M = np.eye(4)
            M[:3, :3] = _rpy_to_R(*j["rpy"])
            M[:3, 3] = j["xyz"]
            T = T @ M
            if j["type"] == "revolute":
                q = 0.0
                if j["name"] in self.index:
                    q = math.radians(q_deg[self.index[j["name"]]])
                Rj = np.eye(4)
                Rj[:3, :3] = _axis_R(j["axis"], q)
                T = T @ Rj
        return T

    def forward_kinematics(self, q_deg):
        return self._fk(q_deg[: len(self.names)])

    def _jacobian(self, q_deg, eps_deg=0.2):
        base = self._fk(q_deg)
        J = np.zeros((6, len(self.names)))
        for i in range(len(self.names)):
            q2 = q_deg.copy()
            q2[i] += eps_deg
            T2 = self._fk(q2)
            eps = math.radians(eps_deg)
            J[:3, i] = (T2[:3, 3] - base[:3, 3]) / eps
            J[3:, i] = _rotvec(T2[:3, :3] @ base[:3, :3].T) / eps
        return J

    def inverse_kinematics(self, current_deg, desired, position_weight=1.0,
                           orientation_weight=0.3, max_iters=10):
        q = np.array(current_deg[: len(self.names)], dtype=float)
        lo = np.degrees([j["lo"] for j in self.chain if j["name"] in self.index])
        hi = np.degrees([j["hi"] for j in self.chain if j["name"] in self.index])
        order = [self.index[j["name"]] for j in self.chain if j["name"] in self.index]
        lo = lo[np.argsort(order)]
        hi = hi[np.argsort(order)]

        for _ in range(max_iters):
            T = self._fk(q)
            e_pos = desired[:3, 3] - T[:3, 3]
            e_rot = _rotvec(desired[:3, :3] @ T[:3, :3].T)
            e = np.concatenate([position_weight * e_pos, orientation_weight * e_rot])
            if np.linalg.norm(e_pos) < 0.2 and np.linalg.norm(e_rot) < 0.004:
                break
            J = self._jacobian(q)
            J[:3] *= position_weight
            J[3:] *= orientation_weight
            lam = 0.08
            dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(6), e)
            dq = np.clip(dq, -math.radians(8), math.radians(8))
            q = np.clip(q + np.degrees(dq), lo, hi)
        return q
