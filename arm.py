"""机械臂后端：SO-101（LeRobot 总线 + placo 逆解）和一个用于无硬件联调的 Mock。"""

from pathlib import Path

import numpy as np


class SO101Arm:
    def __init__(self, cfg):
        from lerobot.model.kinematics import RobotKinematics
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        self.names = list(cfg["joints"])
        self.urdf = cfg["urdf"]

        self.robot = SO101Follower(SO101FollowerConfig(port=cfg["port"]))
        self.robot.connect()
        self.kin = RobotKinematics(
            self.urdf, target_frame_name="gripper_frame_link", joint_names=self.names
        )
        self.ori_weight = cfg["orientation_weight"]
        self.ik_iters = cfg["ik_iters"]
        self.max_step = cfg["max_joint_step_deg"]
        self.q = self.read()
        self._gripper = None

    def read(self):
        obs = self.robot.get_observation()
        self._gripper = obs.get("gripper.pos")
        self.q = np.array([obs[f"{n}.pos"] for n in self.names], dtype=float)
        return self.q.copy()

    def fk(self, q=None):
        return self.kin.forward_kinematics(self.q if q is None else q)

    def move_to(self, pose):
        target = self.kin.inverse_kinematics(
            self.q,
            pose,
            position_weight=1.0,
            orientation_weight=self.ori_weight,
            max_iters=self.ik_iters,
        )
        target = self.q + np.clip(target - self.q, -self.max_step, self.max_step)
        action = {f"{n}.pos": float(v) for n, v in zip(self.names, target)}
        if self._gripper is not None:
            action["gripper.pos"] = float(self._gripper)
        self.robot.send_action(action)
        self.q = target

    def hold(self):
        action = {f"{n}.pos": float(v) for n, v in zip(self.names, self.q)}
        if self._gripper is not None:
            action["gripper.pos"] = float(self._gripper)
        self.robot.send_action(action)

    def release(self):
        self.robot.disconnect()


class MockArm:
    """没有硬件时用来验证状态机和坐标逻辑：直接积分位姿，不做逆解。"""

    def __init__(self, pose=None):
        self.pose = np.eye(4) if pose is None else np.array(pose, dtype=float)
        self.calls = 0

    def read(self):
        return np.zeros(5)

    def fk(self, q=None):
        return self.pose.copy()

    def move_to(self, pose):
        self.pose = np.array(pose, dtype=float)
        self.calls += 1

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
