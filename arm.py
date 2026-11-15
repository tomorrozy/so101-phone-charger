"""机械臂后端：SO-101（LeRobot 总线 + placo 逆解）和一个用于无硬件联调的 Mock。"""

from pathlib import Path

import numpy as np


class SO101Arm:
    def __init__(self, cfg):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        self.names = list(cfg["joints"])
        self.urdf = cfg["urdf"]

        self.robot = SO101Follower(SO101FollowerConfig(port=cfg["port"]))
        self.robot.connect()
        try:
            from lerobot.model.kinematics import RobotKinematics
            self.kin = RobotKinematics(
                self.urdf, target_frame_name="gripper_frame_link", joint_names=self.names
            )
            self.kin_mm = False   # placo 用米
        except Exception:
            from kinematics_np import SimpleKinematics
            self.kin = SimpleKinematics(
                self.urdf, self.names, target_frame="gripper_frame_link"
            )
            self.kin_mm = True    # 内置解用毫米
        self.ori_weight = cfg["orientation_weight"]
        self.ik_iters = cfg["ik_iters"]
        self.max_step = cfg["max_joint_step_deg"]
        self.q = self.read()
        self._gripper = None
        self._fallback_seed = np.array([0.0, -30.0, -40.0, 60.0, 0.0])

    def read(self):
        obs = self.robot.get_observation()
        self._gripper = obs.get("gripper.pos")
        self.q = np.array([obs[f"{n}.pos"] for n in self.names], dtype=float)
        return self.q.copy()

    def fk(self, q=None):
        T = self.kin.forward_kinematics(self.q if q is None else q)
        if not self.kin_mm:
            T = T.copy()
            T[:3, 3] *= 1000.0
        return T

    def load(self):
        """读舵机负载（原始单位，约 0-1000）。读不到返回 None。"""
        for reg in ("Present_Load", "Present_Current"):
            try:
                vals = self.robot.bus.sync_read(reg, normalize=False)
                return max(abs(float(v)) for v in vals.values())
            except Exception:
                continue
        return None

    def move_to(self, pose):
        pose_m = pose
        if not self.kin_mm:
            pose_m = pose.copy()
            pose_m[:3, 3] /= 1000.0
        target = self.kin.inverse_kinematics(
            self.q,
            pose_m,
            position_weight=1.0,
            orientation_weight=self.ori_weight,
            max_iters=self.ik_iters,
        )
        if self._pos_err(target, pose_m) > 3.0 and len(self.names) == 5:
            alt = self.kin.inverse_kinematics(
                self._fallback_seed,
                pose_m,
                position_weight=1.0,
                orientation_weight=self.ori_weight,
                max_iters=max(self.ik_iters * 2, 20),
            )
            if self._pos_err(alt, pose_m) < self._pos_err(target, pose_m):
                target = alt
        target = self.q + np.clip(target - self.q, -self.max_step, self.max_step)
        action = {f"{n}.pos": float(v) for n, v in zip(self.names, target)}
        if self._gripper is not None:
            action["gripper.pos"] = float(self._gripper)
        self.robot.send_action(action)
        self.q = target

    def _pos_err(self, q_deg, pose_native):
        T = self.kin.forward_kinematics(q_deg)
        return float(np.linalg.norm(T[:3, 3] - pose_native[:3, 3]))

    def hold(self):
        action = {f"{n}.pos": float(v) for n, v in zip(self.names, self.q)}
        if self._gripper is not None:
            action["gripper.pos"] = float(self._gripper)
        self.robot.send_action(action)

    def release(self):
        self.robot.disconnect()


class MockArm:
    """没有硬件时用来验证状态机和坐标逻辑：直接积分位姿，不做逆解。"""

    def __init__(self, pose=None, load_value=None):
        self.pose = np.eye(4) if pose is None else np.array(pose, dtype=float)
        self.load_value = load_value
        self.calls = 0

    def read(self):
        return np.zeros(5)

    def fk(self, q=None):
        return self.pose.copy()

    def load(self):
        return self.load_value

    def move_to(self, pose):
        self.pose = np.array(pose, dtype=float)
        self.calls += 1

    def _pos_err(self, q_deg, pose_native):
        T = self.kin.forward_kinematics(q_deg)
        return float(np.linalg.norm(T[:3, 3] - pose_native[:3, 3]))

    def hold(self):
        pass

    def release(self):
        pass


def make_arm(cfg, mock=False):
    if mock:
        return MockArm()
    backend = cfg["backend"].lower()
    if backend != "so101":
        raise ValueError(f"没有实现的后端: {backend}（目前只有 so101）")
    cfg = dict(cfg)
    cfg["urdf"] = str(Path(cfg["urdf"]))
    return SO101Arm(cfg)
