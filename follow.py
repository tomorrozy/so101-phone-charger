"""跟随控制器 + 状态机 + 安全约束。

命令对象是末端位姿；盘面到手机背面的"间隙 gap"由状态机控制：
hover 60mm 跟随，静止后收到 3mm 贴合，手机一动就退回。
"""

import time
from enum import Enum
from pathlib import Path

import cv2
import numpy as np

from arm import make_arm
from tracker import MarkerTracker, Target, inv_T, load_yaml, make_T


class State(Enum):
    IDLE = "IDLE"
    FOLLOW = "FOLLOW"
    APPROACH = "APPROACH"
    CHARGE = "CHARGE"
    CARRY = "CARRY"
    TRANSPORT = "TRANSPORT"
    RELEASE = "RELEASE"
    RETRACT = "RETRACT"


class Charger:
    """Qi 使能。接继电器/MOSFET 时把这里换成 GPIO 或串口命令。"""

    def __init__(self):
        self.on = False

    def enable(self):
        self.on = True

    def disable(self):
        self.on = False

    def power_w(self):
        return None


def rot_between(a, b):
    """把向量 a 转到 b 的最短旋转矩阵。"""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-8:
        return np.eye(3)
    if c < -1.0 + 1e-8:
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        return cv2.Rodrigues(axis / np.linalg.norm(axis) * np.pi)[0]
    axis = v / np.linalg.norm(v)
    return cv2.Rodrigues(axis * np.arctan2(np.linalg.norm(v), c))[0]


class Follower:
    def __init__(self, cfg, arm, dock, charger=None, deliver_T=None):
        self.cfg = cfg
        self.arm = arm
        self.charger = charger or Charger()

        self.R0 = dock["T_base_ee0"][:3, :3]
        self.n0 = dock["normal0"] / np.linalg.norm(dock["normal0"])
        self.d0 = float(dock["d0_mm"])

        c = cfg["control"]
        s = cfg["state"]
        self.vmax = c["vmax_mm_s"]
        self.omax = np.deg2rad(90.0)
        self.hover = cfg["puck"]["hover_mm"]
        self.contact = cfg["puck"]["contact_mm"]
        self.contact_rot = cfg["puck"].get("contact_mm_rotated", self.contact)
        self.rot_cos = cfg["puck"].get("rotated_axis_cos", 0.87)
        self.approach_v = cfg["puck"]["approach_mm_s"]
        self.retract_len = cfg["puck"]["retract_mm"]
        self.settle_speed = s["settle_speed_mm_s"]
        self.settle_time = s["settle_time_s"]
        self.charge_confirm = s["charge_confirm_s"]
        self.release_speed = s["release_speed_mm_s"]
        self.lost_timeout = s["lost_timeout_s"]
        carry = cfg.get("carry", {})
        self.carry_on = bool(carry.get("enabled", False))
        self.carry_gap = float(carry.get("gap_mm", 1.0))
        self.carry_settle = float(carry.get("settle_s", 0.8))
        self.slip_mm = float(carry.get("slip_mm", 3.0))
        self.slip_deg = float(carry.get("slip_deg", 5.0))
        self.carry_target = None
        self.carry_ref = None
        self.carry_ready_t = None
        tr = cfg.get("transport", {})
        self.transport_on = bool(tr.get("enabled", False))
        self.transport_speed = float(tr.get("speed_mm_s", 80.0))
        self.transport_delay = float(tr.get("delay_s", 1.5))
        self.deliver_T = deliver_T
        self.deliver_xyz = np.array(tr.get("deliver_mm", [300.0, -120.0, 220.0]), dtype=float)
        self.transported = False
        self.transport_start = None
        self.transport_end = None
        self.transport_t0 = 0.0
        self.transport_T = 1.0

        self.state = State.IDLE
        self.t_state = time.monotonic()
        self.gap = self.hover
        self.mode = "portrait"
        self.release_reason = ""
        self.force_retract = False
        self.still_t = 0.0
        self.lost_t = 0.0
        self.lost_handled = False
        self.at_contact_since = None
        self.charge_origin = None
        self.last_n = self.n0.copy()
        self.setpoint = None
        self.retract_target = None
        self.events = []

    def _enter(self, state):
        if state is not self.state:
            self.events.append((time.monotonic(), self.state.value, state.value))
            self.state = state
            self.t_state = time.monotonic()
            if state is State.FOLLOW:
                self.still_t = 0.0
                self.at_contact_since = None

    def _contact_gap(self, target):
        """手机长轴是否朝相机侧：横放时多留 3mm，避免手机长边贴到腕部相机。"""
        R = rot_between(self.n0, target.normal) @ self.R0
        c = 0.0 if target.top_dir is None else abs(float(np.dot(R[:, 0], target.top_dir)))
        rotated = c >= self.rot_cos
        self.mode = "landscape" if rotated else "portrait"
        return self.contact_rot if rotated else self.contact

    def _rel(self, target, ee_pose):
        T = make_T(target.T_base_marker[:3, :3], target.pos_mm)
        return inv_T(ee_pose) @ T

    def _slip_exceeded(self, rel):
        if self.carry_ref is None:
            return False
        dp = float(np.linalg.norm(rel[:3, 3] - self.carry_ref[:3, 3]))
        Rerr = rel[:3, :3] @ self.carry_ref[:3, :3].T
        ang = float(np.degrees(np.arccos(np.clip((np.trace(Rerr) - 1) / 2, -1, 1))))
        self.last_slip = (dp, ang)
        return dp > self.slip_mm or ang > self.slip_deg

    def _abort(self, target, reason):
        self.charger.disable()
        self.release_reason = reason
        self.force_retract = True
        self.retract_target = self.setpoint.copy()
        self.retract_target[:3, 3] += target.normal * self.retract_len
        self._enter(State.RETRACT)

    def set_carry(self, on):
        self.carry_on = bool(on)
        self.transported = False
        if not self.carry_on and self.state is State.CARRY:
            self.charger.disable()
            self.carry_target = None
            self.carry_ref = None
            self._enter(State.RELEASE)

    def request_transport(self):
        if self.carry_on and self.state is State.CARRY and self.carry_ref is not None:
            self._start_transport(time.monotonic())

    def _start_transport(self, now):
        self.transport_start = self.setpoint.copy()
        if self.deliver_T is not None:
            self.transport_end = self.deliver_T.copy()
        else:
            self.transport_end = self.setpoint.copy()
            self.transport_end[:3, 3] = self.deliver_xyz
        self.transport_end[:3, 3] = self._clamp_box(self.transport_end[:3, 3])
        dist = float(np.linalg.norm(self.transport_end[:3, 3] - self.setpoint[:3, 3]))
        self.transport_t0 = now
        self.transport_T = max(1.2, dist / max(self.transport_speed * 0.6, 1e-3))
        self._enter(State.TRANSPORT)

    def _desired(self, target, gap):
        d = self.d0 + (gap - self.contact)
        pos = target.pos_mm + target.normal * d
        R = rot_between(self.n0, target.normal) @ self.R0
        return make_T(R, pos)

    def _clamp_box(self, p):
        return np.array([
            float(np.clip(p[0], *self.cfg["safety"]["workspace_mm"]["x"])),
            float(np.clip(p[1], *self.cfg["safety"]["workspace_mm"]["y"])),
            float(np.clip(p[2], *self.cfg["safety"]["workspace_mm"]["z"])),
        ])

    def _towards(self, cur, dst, dt):
        dp = np.clip(dst[:3, 3] - cur[:3, 3], -self.vmax * dt, self.vmax * dt)
        pos = cur[:3, 3] + dp
        rvec, _ = cv2.Rodrigues(dst[:3, :3] @ cur[:3, :3].T)
        ang = np.linalg.norm(rvec)
        if ang > self.omax * dt:
            rvec = rvec * (self.omax * dt / ang)
        R = cur[:3, :3] @ cv2.Rodrigues(rvec)[0]
        return make_T(R, pos)

    def step(self, target, dt):
        now = time.monotonic()
        self.events = []

        if self.setpoint is None:
            self.setpoint = self.arm.fk()

        if not target.visible:
            self.charger.disable()
            self.lost_t += dt
            if not self.lost_handled and self.lost_t > self.lost_timeout:
                self.lost_handled = True
                self.retract_target = self.setpoint.copy()
                self.retract_target[:3, 3] += self.last_n * self.retract_len
                self._enter(State.RETRACT)
            if self.state is State.RETRACT:
                self.setpoint = self._towards(self.setpoint, self.retract_target, dt)
                if np.linalg.norm(self.setpoint[:3, 3] - self.retract_target[:3, 3]) < 1.0:
                    self._enter(State.IDLE)
            self.arm.move_to(self.setpoint)
            return self.setpoint

        self.lost_t = 0.0
        self.lost_handled = False
        self.last_n = target.normal.copy()

        if self.state is State.RETRACT and self.force_retract:
            self.setpoint = self._towards(self.setpoint, self.retract_target, dt)
            if np.linalg.norm(self.setpoint[:3, 3] - self.retract_target[:3, 3]) < 1.0:
                self.force_retract = False
                self._enter(State.IDLE)
            self.arm.move_to(self.setpoint)
            return self.setpoint

        if self.state in (State.IDLE, State.RETRACT):
            self.gap = self.hover
            self.still_t = 0.0
            self.at_contact_since = None
            self._enter(State.FOLLOW)

        if self.state is State.FOLLOW:
            self.gap = self.hover
            self.still_t = self.still_t + dt if target.speed_mm_s < self.settle_speed else 0.0
            if self.still_t >= self.settle_time:
                self._enter(State.APPROACH)

        elif self.state is State.APPROACH:
            gap_c = self._contact_gap(target)
            self.gap = max(gap_c, self.gap - self.approach_v * dt)
            if target.speed_mm_s > self.release_speed:
                self.still_t = 0.0
                self.at_contact_since = None
                self._enter(State.FOLLOW)
            elif self.gap <= gap_c + 1e-3:
                if self.at_contact_since is None:
                    self.at_contact_since = now
                elif now - self.at_contact_since >= self.charge_confirm:
                    self.charge_origin = target.pos_mm.copy()
                    self.charger.enable()
                    if self.carry_on:
                        self.carry_target = None
                        self.carry_ref = None
                        self.carry_ready_t = None
                        self.transported = False
                        self._enter(State.CARRY)
                    else:
                        self._enter(State.CHARGE)
            else:
                self.at_contact_since = None

        elif self.state is State.CHARGE:
            self.gap = self._contact_gap(target)
            if (target.speed_mm_s > self.release_speed
                    or np.linalg.norm(target.pos_mm - self.charge_origin) > 5.0):
                self.charger.disable()
                self._enter(State.RELEASE)

        elif self.state is State.CARRY:
            self.gap = self.carry_gap
            if self.carry_target is None:
                self.carry_target = self._desired(target, self.carry_gap)
            self.setpoint = self._towards(self.setpoint, self.carry_target, dt)
            self.arm.move_to(self.setpoint)
            if now - self.t_state >= self.carry_settle and target.T_base_marker is not None:
                rel = self._rel(target, self.setpoint)
                if self.carry_ref is None:
                    self.carry_ref = rel
                    self.carry_ready_t = now
                elif self._slip_exceeded(rel):
                    dp, ang = self.last_slip
                    self._abort(target, f"slip {dp:.1f}mm/{ang:.1f}deg")
                    return self.setpoint
                elif (self.transport_on and not self.transported
                      and now - self.carry_ready_t >= self.transport_delay):
                    self._start_transport(now)
            return self.setpoint

        elif self.state is State.TRANSPORT:
            if target.T_base_marker is not None and self.carry_ref is not None:
                if self._slip_exceeded(self._rel(target, self.setpoint)):
                    dp, ang = self.last_slip
                    self._abort(target, f"slip {dp:.1f}mm/{ang:.1f}deg")
                    return self.setpoint
            u = min(1.0, max(0.0, (now - self.transport_t0) / max(self.transport_T, 1e-3)))
            s = u * u * (3.0 - 2.0 * u)
            pos = self.transport_start[:3, 3] + (self.transport_end[:3, 3] - self.transport_start[:3, 3]) * s
            self.setpoint = make_T(self.transport_start[:3, :3], pos)
            self.setpoint[:3, 3] = self._clamp_box(self.setpoint[:3, 3])
            self.arm.move_to(self.setpoint)
            if u >= 1.0:
                self.transported = True
                self.carry_target = self.setpoint.copy()
                self._enter(State.CARRY)
            return self.setpoint

        elif self.state is State.RELEASE:
            self.charger.disable()
            self.gap = min(self.hover, self.gap + 30.0 * dt)
            if self.gap >= self.hover - 1e-3:
                self._enter(State.FOLLOW)

        desired = self._desired(target, self.gap)
        desired[:3, 3] = self._clamp_box(desired[:3, 3])
        self.setpoint = self._towards(self.setpoint, desired, dt)
        self.arm.move_to(self.setpoint)
        return self.setpoint


class ScriptedTracker:
    """mock 模式：模拟手举手机移动，不依赖相机。"""

    def __init__(self, center, normal=(0, 0, 1), top=(0, 1, 0), attach=True):
        self.center = np.array(center, dtype=float)
        self.normal = np.array(normal, dtype=float)
        self.top = np.array(top, dtype=float)
        self.t0 = time.monotonic()
        self.attach = attach
        self.attached = False
        self.rel = None

    def update(self, frame, T_base_ee):
        t = time.monotonic() - self.t0
        if self.attach and t > 4.0 and not self.attached and T_base_ee is not None:
            T_phone = make_T(np.column_stack([np.cross(self.top, self.normal), self.top, self.normal]),
                             self.center)
            self.rel = inv_T(T_base_ee) @ T_phone
            self.attached = True
        if self.attached:
            T = T_base_ee @ self.rel
            return Target(True, T[:3, 3].copy(), T[:3, 2].copy(), 0.0,
                          time.monotonic(), T[:3, 1].copy(), T), None
        if t < 2.0:
            pos = self.center + np.array([0.0, 120.0 * np.sin(t * 2.0), 20.0 * np.sin(t * 3.0)])
            speed = float(np.hypot(240.0 * np.cos(t * 2.0), 60.0 * np.cos(t * 3.0)))
        elif t < 9.0:
            pos = self.center
            speed = 0.0
        elif t < 10.0:
            pos = self.center + np.array([0.0, 40.0 * (t - 9.0), 0.0])
            speed = 40.0
        else:
            pos = self.center + np.array([0.0, 40.0, 0.0])
            speed = 0.0
        x = np.cross(self.top, self.normal)
        T = make_T(np.column_stack([x, self.top, self.normal]), pos)
        return Target(True, pos, self.normal.copy(), speed, time.monotonic(), self.top, T), None


def run(cfg, config_dir=".", mock=False, duration=None):
    base = Path(config_dir)
    arm = make_arm(cfg["arm"], mock=mock)
    raw = load_yaml(base / cfg["dock"])
    dock = {
        "T_base_ee0": np.array(raw["T_base_ee0"], dtype=float).reshape(4, 4),
        "normal0": np.array(raw["normal0"], dtype=float),
        "d0_mm": raw["d0_mm"],
    }
    deliver_T = None
    deliver_path = base / "calib" / "deliver.yaml"
    if deliver_path.exists():
        deliver_T = np.array(load_yaml(deliver_path)["T_base_ee"], dtype=float).reshape(4, 4)
    follower = Follower(cfg, arm, dock, deliver_T=deliver_T)

    if mock:
        tracker = ScriptedTracker(center=(260.0, 0.0, 260.0),
                                  attach=bool(cfg.get("carry", {}).get("enabled", False)))
        cap = None
    else:
        cap = cv2.VideoCapture(cfg["camera"]["index"])
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["camera"]["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["camera"]["height"])
        cap.set(cv2.CAP_PROP_FPS, cfg["camera"]["fps"])
        tracker = MarkerTracker(cfg, config_dir)

    dt = 1.0 / cfg["control"]["hz"]
    t0 = time.monotonic()
    last = t0
    try:
        while duration is None or time.monotonic() - t0 < duration:
            if cap is not None:
                ok, frame = cap.read()
                if not ok:
                    break
                T_base_ee = arm.fk(arm.read())
                target, corners = tracker.update(frame, T_base_ee)
            else:
                target, corners = tracker.update(None, arm.fk())

            follower.step(target, dt)
            for ev in follower.events:
                print(f"[{ev[0] - t0:6.2f}s] {ev[1]} -> {ev[2]}")

            if cap is not None:
                if corners is not None:
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                tag = "CARRY" if follower.carry_on else "FOLLOW"
                cv2.putText(frame, f"{tag} {follower.state.value} {follower.mode} {follower.gap:.0f}mm",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)
                cv2.putText(frame, "c: follow/carry   t: transport   esc: quit",
                            (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
                cv2.imshow("follow", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break
                if key == ord("c"):
                    follower.set_carry(not follower.carry_on)
                    print("carry mode:", follower.carry_on)
                if key == ord("t"):
                    follower.request_transport()

            sleep = dt - (time.monotonic() - last)
            if sleep > 0:
                time.sleep(sleep)
            last = time.monotonic()
    finally:
        follower.charger.disable()
        if cap is not None:
            cap.release()
            cv2.destroyAllWindows()
        arm.release()


if __name__ == "__main__":
    import argparse

    import yaml

    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()

    path = Path(args.config)
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    run(cfg, config_dir=path.parent, mock=args.mock, duration=args.seconds)
