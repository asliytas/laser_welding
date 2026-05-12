import argparse
import json
import math
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import Optional

try:
    from trajectory_mapping_1A import (
        WeldPoint,
        WeldTrajectory,
        load_trajectory_json,
    )
except Exception:
    WeldPoint = None
    WeldTrajectory = None
    load_trajectory_json = None


@dataclass
class TCPPose:
    """Tool Center Point pose."""
    position: tuple
    rotation_matrix: list
    quaternion: tuple
    euler_zyx_deg: tuple


@dataclass
class TrajectorySetpoint:
    """Timed, posed, and process-aware robot setpoint."""
    index: int
    timestamp: float
    arc_length: float
    phase: str
    pose: TCPPose
    velocity: float
    acceleration: float
    laser_power: float
    wire_feed_rate: float
    shielding_gas_flow: float
    focus_offset: float
    notes: list = field(default_factory=list)

    def to_dict(self):
        return {
            "index": self.index,
            "timestamp": self.timestamp,
            "arc_length": self.arc_length,
            "phase": self.phase,
            "pose": {
                "position": list(self.pose.position),
                "rotation_matrix": self.pose.rotation_matrix,
                "quaternion": list(self.pose.quaternion),
                "euler_zyx_deg": list(self.pose.euler_zyx_deg),
            },
            "velocity": self.velocity,
            "acceleration": self.acceleration,
            "laser_power": self.laser_power,
            "wire_feed_rate": self.wire_feed_rate,
            "shielding_gas_flow": self.shielding_gas_flow,
            "focus_offset": self.focus_offset,
            "notes": list(self.notes),
        }


@dataclass
class MotionConfig:
    target_speed_mm_s: float = 20.0
    acceleration_max_mm_s2: float = 200.0
    jerk_max_mm_s3: float = 2000.0
    profile: str = "trapezoidal"


@dataclass
class TCPConfig:
    strategy: str = "bisector_normal"
    flip_torch_z: bool = False
    custom_world_z: tuple = (0.0, 0.0, 1.0)


@dataclass
class ApproachRetractConfig:
    distance_mm: float = 20.0
    speed_mm_s: float = 50.0
    direction: str = "normal_positive"
    custom_vector: Optional[tuple] = None
    n_points: int = 5


@dataclass
class ProcessConfig:
    laser_power_w: float = 1500.0
    wire_feed_rate: float = 5.0
    shielding_gas_flow: float = 15.0
    focus_offset_mm: float = 0.0
    ramp_up_time_s: float = 0.2
    ramp_down_time_s: float = 0.2


@dataclass
class WeldProgram:
    weld_id: int
    weld_name: str
    motion: MotionConfig
    tcp: TCPConfig
    approach: ApproachRetractConfig
    retract: ApproachRetractConfig
    process: ProcessConfig
    setpoints: list = field(default_factory=list)
    total_duration_s: float = 0.0
    weld_duration_s: float = 0.0
    notes: list = field(default_factory=list)

    def to_dict(self):
        return {
            "weld_id": self.weld_id,
            "weld_name": self.weld_name,
            "motion": asdict(self.motion),
            "tcp": asdict(self.tcp),
            "approach": asdict(self.approach),
            "retract": asdict(self.retract),
            "process": asdict(self.process),
            "setpoints": [s.to_dict() for s in self.setpoints],
            "total_duration_s": self.total_duration_s,
            "weld_duration_s": self.weld_duration_s,
            "notes": list(self.notes),
        }


def _v_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_mul(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v):
    return math.sqrt(max(0.0, _dot(v, v)))


def _normalize(v, default=(1.0, 0.0, 0.0), eps=1e-9):
    n = _norm(v)
    if n < eps:
        return default, False
    return (v[0] / n, v[1] / n, v[2] / n), True


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _angle_between(a, b):
    na, ok_a = _normalize(a)
    nb, ok_b = _normalize(b)
    if not ok_a or not ok_b:
        return 0.0
    return math.degrees(math.acos(_clamp(_dot(na, nb), -1.0, 1.0)))


def rotation_matrix_to_quaternion(R):
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    q, _ = _normalize((qw, qx, qy), default=(1.0, 0.0, 0.0))
    scale = math.sqrt(max(1e-12, qw * qw + qx * qx + qy * qy + qz * qz))
    return (qw / scale, qx / scale, qy / scale, qz / scale)


def rotation_matrix_to_euler_zyx(R):
    if abs(R[2][0]) < 1.0 - 1e-9:
        ry = math.asin(-R[2][0])
        cy = math.cos(ry)
        rx = math.atan2(R[2][1] / cy, R[2][2] / cy)
        rz = math.atan2(R[1][0] / cy, R[0][0] / cy)
    else:
        ry = math.pi / 2 if R[2][0] <= -1.0 else -math.pi / 2
        rx = 0.0
        rz = math.atan2(-R[0][1], R[1][1])
    return (math.degrees(rz), math.degrees(ry), math.degrees(rx))


def compute_tcp_frame(position, tangent, normal, tcp_config=None):
    tcp_config = tcp_config or TCPConfig()
    notes = []

    x_axis, ok_x = _normalize(tangent, default=(1.0, 0.0, 0.0))
    if not ok_x:
        notes.append("zero tangent, used world X fallback")

    if tcp_config.strategy == "world_z_up":
        z_source = tcp_config.custom_world_z
    elif tcp_config.strategy == "custom":
        z_source = tcp_config.custom_world_z
    else:
        z_source = normal

    z_axis, ok_z = _normalize(z_source, default=(0.0, 0.0, 1.0))
    if not ok_z:
        notes.append("zero normal, used world Z fallback")

    if not tcp_config.flip_torch_z:
        z_axis = _v_mul(z_axis, -1.0)

    x_axis = _v_sub(x_axis, _v_mul(z_axis, _dot(x_axis, z_axis)))
    x_axis, ok_x2 = _normalize(x_axis, default=(1.0, 0.0, 0.0))
    if not ok_x2:
        notes.append("tangent parallel to torch Z, used fallback orthogonal axis")
        fallback = (0.0, 1.0, 0.0) if abs(z_axis[0]) > 0.8 else (1.0, 0.0, 0.0)
        x_axis, _ = _normalize(_cross(fallback, z_axis))

    y_axis, ok_y = _normalize(_cross(z_axis, x_axis), default=(0.0, 1.0, 0.0))
    if not ok_y:
        notes.append("invalid TCP Y axis fallback")
    z_axis, _ = _normalize(_cross(x_axis, y_axis), default=z_axis)

    R = [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]
    return TCPPose(
        position=tuple(position),
        rotation_matrix=R,
        quaternion=rotation_matrix_to_quaternion(R),
        euler_zyx_deg=rotation_matrix_to_euler_zyx(R),
    ), notes


def plan_trapezoidal(arc_lengths, v_target, a_max):
    notes = []
    if not arc_lengths:
        return [], notes
    if v_target <= 0 or a_max <= 0:
        raise ValueError("target speed and acceleration must be positive")

    total = max(0.0, arc_lengths[-1])
    if total <= 1e-9:
        return [(0.0, 0.0, 0.0) for _ in arc_lengths], notes

    s_acc_target = v_target * v_target / (2.0 * a_max)
    if 2.0 * s_acc_target > total:
        v_peak = math.sqrt(a_max * total)
        s_acc = total / 2.0
        s_const = 0.0
        notes.append(
            f"weld too short for target speed, reaching peak v={v_peak:.2f} mm/s"
        )
    else:
        v_peak = v_target
        s_acc = s_acc_target
        s_const = total - 2.0 * s_acc

    t_acc = v_peak / a_max
    t_const = s_const / v_peak if v_peak > 1e-9 else 0.0
    out = []
    for s in arc_lengths:
        s = _clamp(s, 0.0, total)
        if s <= s_acc or total <= 1e-9:
            v = math.sqrt(max(0.0, 2.0 * a_max * s))
            t = v / a_max
            a = a_max if v > 1e-9 else 0.0
        elif s <= s_acc + s_const:
            v = v_peak
            t = t_acc + (s - s_acc) / v_peak
            a = 0.0
        else:
            remaining = max(0.0, total - s)
            v = math.sqrt(max(0.0, 2.0 * a_max * remaining))
            t = t_acc + t_const + (v_peak - v) / a_max
            a = -a_max if v > 1e-9 else 0.0
        out.append((t, v, a))
    return out, notes


def plan_s_curve(arc_lengths, v_target, a_max, j_max):
    profile, notes = plan_trapezoidal(arc_lengths, v_target, a_max)
    notes.append("S-curve fallback to trapezoidal in planner 1.A")
    return profile, notes


def _direction_from_config(normal, config):
    if config.direction == "world_z":
        return (0.0, 0.0, 1.0)
    if config.direction == "custom" and config.custom_vector is not None:
        return _normalize(config.custom_vector, default=(0.0, 0.0, 1.0))[0]
    return _normalize(normal, default=(0.0, 0.0, 1.0))[0]


def _process_zero(process):
    return {
        "laser_power": 0.0,
        "wire_feed_rate": 0.0,
        "shielding_gas_flow": process.shielding_gas_flow,
        "focus_offset": process.focus_offset_mm,
    }


def _make_setpoint(index, timestamp, arc_length, phase, pose, velocity, acceleration, process_values, notes=None):
    return TrajectorySetpoint(
        index=index,
        timestamp=timestamp,
        arc_length=arc_length,
        phase=phase,
        pose=pose,
        velocity=velocity,
        acceleration=acceleration,
        laser_power=process_values["laser_power"],
        wire_feed_rate=process_values["wire_feed_rate"],
        shielding_gas_flow=process_values["shielding_gas_flow"],
        focus_offset=process_values["focus_offset"],
        notes=list(notes or []),
    )


def compute_approach_points(first_weld_setpoint, normal_at_start, config, process, start_index=0):
    direction = _direction_from_config(normal_at_start, config)
    n_points = max(2, int(config.n_points))
    start_pos = first_weld_setpoint.pose.position
    start_time = -config.distance_mm / max(config.speed_mm_s, 1e-9)
    points = []
    for i in range(n_points):
        alpha = i / (n_points - 1)
        offset = config.distance_mm * (1.0 - alpha)
        pos = _v_add(start_pos, _v_mul(direction, offset))
        pose = TCPPose(
            position=pos,
            rotation_matrix=first_weld_setpoint.pose.rotation_matrix,
            quaternion=first_weld_setpoint.pose.quaternion,
            euler_zyx_deg=first_weld_setpoint.pose.euler_zyx_deg,
        )
        points.append(_make_setpoint(
            start_index + i,
            start_time * (1.0 - alpha),
            -offset,
            "approach",
            pose,
            config.speed_mm_s,
            0.0,
            _process_zero(process),
        ))
    return points


def compute_retract_points(last_weld_setpoint, normal_at_end, config, process, start_index=0):
    direction = _direction_from_config(normal_at_end, config)
    n_points = max(2, int(config.n_points))
    start_pos = last_weld_setpoint.pose.position
    points = []
    base_time = last_weld_setpoint.timestamp
    for i in range(1, n_points + 1):
        alpha = i / n_points
        offset = config.distance_mm * alpha
        pos = _v_add(start_pos, _v_mul(direction, offset))
        pose = TCPPose(
            position=pos,
            rotation_matrix=last_weld_setpoint.pose.rotation_matrix,
            quaternion=last_weld_setpoint.pose.quaternion,
            euler_zyx_deg=last_weld_setpoint.pose.euler_zyx_deg,
        )
        points.append(_make_setpoint(
            start_index + i - 1,
            base_time + offset / max(config.speed_mm_s, 1e-9),
            last_weld_setpoint.arc_length + offset,
            "retract",
            pose,
            config.speed_mm_s,
            0.0,
            _process_zero(process),
        ))
    return points


def _process_values_at(timestamp, weld_duration, process):
    ramp_up = max(0.0, process.ramp_up_time_s)
    ramp_down = max(0.0, process.ramp_down_time_s)
    scale = 1.0
    if ramp_up > 1e-9 and timestamp < ramp_up:
        scale = timestamp / ramp_up
    if ramp_down > 1e-9 and timestamp > weld_duration - ramp_down:
        scale = min(scale, max(0.0, (weld_duration - timestamp) / ramp_down))
    scale = _clamp(scale, 0.0, 1.0)
    return {
        "laser_power": process.laser_power_w * scale,
        "wire_feed_rate": process.wire_feed_rate * scale,
        "shielding_gas_flow": process.shielding_gas_flow,
        "focus_offset": process.focus_offset_mm,
    }


def apply_process_profile(weld_setpoints, process_config, weld_total_duration):
    for sp in weld_setpoints:
        values = _process_values_at(sp.timestamp, weld_total_duration, process_config)
        sp.laser_power = values["laser_power"]
        sp.wire_feed_rate = values["wire_feed_rate"]
        sp.shielding_gas_flow = values["shielding_gas_flow"]
        sp.focus_offset = values["focus_offset"]


def _quat_angle_deg(q1, q2):
    dot = abs(sum(q1[i] * q2[i] for i in range(4)))
    return math.degrees(2.0 * math.acos(_clamp(dot, -1.0, 1.0)))


def _radius_from_three_points(a, b, c):
    ab = _v_sub(b, a)
    bc = _v_sub(c, b)
    ac = _v_sub(c, a)
    area2 = _norm(_cross(ab, ac))
    denom = area2
    if denom < 1e-9:
        return float("inf")
    return (_norm(ab) * _norm(bc) * _norm(ac)) / denom


def validate_program(program):
    weld_points = [sp for sp in program.setpoints if sp.phase == "weld"]
    for i in range(len(weld_points) - 1):
        a = weld_points[i]
        b = weld_points[i + 1]
        x_a = (
            a.pose.rotation_matrix[0][0],
            a.pose.rotation_matrix[1][0],
            a.pose.rotation_matrix[2][0],
        )
        x_b = (
            b.pose.rotation_matrix[0][0],
            b.pose.rotation_matrix[1][0],
            b.pose.rotation_matrix[2][0],
        )
        tangent_angle = _angle_between(x_a, x_b)
        if tangent_angle > 30.0:
            program.notes.append(
                f"Setpoint {b.index}: tangent jump {tangent_angle:.1f} deg, sharp corner"
            )
        q_angle = _quat_angle_deg(a.pose.quaternion, b.pose.quaternion)
        if q_angle > 30.0:
            program.notes.append(
                f"Setpoint {b.index}: orientation jump {q_angle:.1f} deg"
            )
        if abs(b.velocity - a.velocity) > 0.5 * max(program.motion.target_speed_mm_s, 1e-9):
            program.notes.append(
                f"Setpoint {b.index}: velocity discontinuity {abs(b.velocity - a.velocity):.1f} mm/s"
            )

    for i in range(1, len(weld_points) - 1):
        radius = _radius_from_three_points(
            weld_points[i - 1].pose.position,
            weld_points[i].pose.position,
            weld_points[i + 1].pose.position,
        )
        if radius < 5.0:
            program.notes.append(
                f"Setpoint {weld_points[i].index}: tight radius {radius:.2f} mm"
            )
    return program.notes


def plan_weld(trajectory, motion=None, tcp=None, approach=None, retract=None, process=None):
    motion = motion or MotionConfig()
    tcp = tcp or TCPConfig()
    approach = approach or ApproachRetractConfig()
    retract = retract or ApproachRetractConfig()
    process = process or ProcessConfig()
    program = WeldProgram(
        weld_id=trajectory.weld_id,
        weld_name=trajectory.weld_name,
        motion=motion,
        tcp=tcp,
        approach=approach,
        retract=retract,
        process=process,
    )

    points = list(trajectory.points)
    if not points:
        program.notes.append("trajectory has no points")
        return program

    arc_lengths = [float(p.arc_length) for p in points]
    if motion.profile == "s_curve":
        profile, notes = plan_s_curve(
            arc_lengths,
            motion.target_speed_mm_s,
            motion.acceleration_max_mm_s2,
            motion.jerk_max_mm_s3,
        )
    else:
        profile, notes = plan_trapezoidal(
            arc_lengths,
            motion.target_speed_mm_s,
            motion.acceleration_max_mm_s2,
        )
    program.notes.extend(notes)

    weld_setpoints = []
    for i, point in enumerate(points):
        pose, pose_notes = compute_tcp_frame(point.position, point.tangent, point.normal, tcp)
        timestamp, velocity, acceleration = profile[i]
        sp = _make_setpoint(
            i,
            timestamp,
            point.arc_length,
            "weld",
            pose,
            velocity,
            acceleration,
            _process_values_at(timestamp, profile[-1][0], process),
            notes=pose_notes,
        )
        weld_setpoints.append(sp)

    weld_duration = profile[-1][0] if profile else 0.0
    apply_process_profile(weld_setpoints, process, weld_duration)

    first_normal = points[0].normal
    last_normal = points[-1].normal
    approach_setpoints = compute_approach_points(
        weld_setpoints[0], first_normal, approach, process, start_index=0
    )
    weld_index_offset = len(approach_setpoints)
    for i, sp in enumerate(weld_setpoints):
        sp.index = weld_index_offset + i

    retract_setpoints = compute_retract_points(
        weld_setpoints[-1],
        last_normal,
        retract,
        process,
        start_index=weld_index_offset + len(weld_setpoints),
    )

    program.setpoints = approach_setpoints + weld_setpoints + retract_setpoints
    for i, sp in enumerate(program.setpoints):
        sp.index = i
    program.weld_duration_s = weld_duration
    program.total_duration_s = max(sp.timestamp for sp in program.setpoints)
    validate_program(program)
    return program


def plan_all_welds(trajectories, motion=None, tcp=None, approach=None, retract=None, process=None):
    return [
        plan_weld(t, motion=motion, tcp=tcp, approach=approach, retract=retract, process=process)
        for t in trajectories
    ]


def _pose_from_dict(data):
    return TCPPose(
        position=tuple(data["position"]),
        rotation_matrix=data["rotation_matrix"],
        quaternion=tuple(data["quaternion"]),
        euler_zyx_deg=tuple(data["euler_zyx_deg"]),
    )


def _setpoint_from_dict(data):
    return TrajectorySetpoint(
        index=data["index"],
        timestamp=data["timestamp"],
        arc_length=data["arc_length"],
        phase=data["phase"],
        pose=_pose_from_dict(data["pose"]),
        velocity=data["velocity"],
        acceleration=data["acceleration"],
        laser_power=data["laser_power"],
        wire_feed_rate=data["wire_feed_rate"],
        shielding_gas_flow=data["shielding_gas_flow"],
        focus_offset=data["focus_offset"],
        notes=data.get("notes", []),
    )


def _program_from_dict(data):
    return WeldProgram(
        weld_id=data["weld_id"],
        weld_name=data["weld_name"],
        motion=MotionConfig(**data.get("motion", {})),
        tcp=TCPConfig(**data.get("tcp", {})),
        approach=ApproachRetractConfig(**data.get("approach", {})),
        retract=ApproachRetractConfig(**data.get("retract", {})),
        process=ProcessConfig(**data.get("process", {})),
        setpoints=[_setpoint_from_dict(s) for s in data.get("setpoints", [])],
        total_duration_s=data.get("total_duration_s", 0.0),
        weld_duration_s=data.get("weld_duration_s", 0.0),
        notes=data.get("notes", []),
    )


def save_program_json(programs, filepath):
    payload = {
        "version": "weld_program_1.A",
        "n_welds": len(programs),
        "programs": [p.to_dict() for p in programs],
    }
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def load_program_json(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        payload = json.load(f)
    return [_program_from_dict(p) for p in payload.get("programs", [])]


def _velocity_color_tuple(v, v_max):
    if v_max <= 1e-9:
        return (0.1, 0.3, 0.9)
    x = _clamp(v / v_max, 0.0, 1.0)
    if x < 0.5:
        k = x / 0.5
        return (0.1, 0.3 + 0.6 * k, 0.9 * (1.0 - k))
    k = (x - 0.5) / 0.5
    return (0.1 + 0.9 * k, 0.9 - 0.15 * k, 0.05)


try:
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QKeySequence
    from PyQt5.QtWidgets import (
        QAction,
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFileDialog,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
    QT_AVAILABLE = True
except Exception as exc:
    QT_IMPORT_ERROR = exc
    QT_AVAILABLE = False


try:
    from OCC.Display.backend import load_backend
    load_backend("pyqt5")
    from OCC.Display.qtDisplay import qtViewer3d
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeVertex
    from OCC.Core.gp import gp_Pnt
    from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
    OCC_AVAILABLE = True
except Exception as exc:
    OCC_IMPORT_ERROR = exc
    OCC_AVAILABLE = False


if QT_AVAILABLE and OCC_AVAILABLE:
    class PanablePlannerViewer(qtViewer3d):
        def __init__(self, parent=None):
            super().__init__(parent)
            self._pan_anchor = None

        def mousePressEvent(self, event):
            if (event.button() == Qt.MiddleButton or
                    (event.button() == Qt.LeftButton and event.modifiers() & Qt.ShiftModifier)):
                self._pan_anchor = event.pos()
                event.accept()
                return
            super().mousePressEvent(event)

        def mouseMoveEvent(self, event):
            if self._pan_anchor is not None:
                dx = event.pos().x() - self._pan_anchor.x()
                dy = event.pos().y() - self._pan_anchor.y()
                self._pan_anchor = event.pos()
                try:
                    self._display.Pan(dx, -dy)
                    self._display.Repaint()
                except Exception:
                    pass
                event.accept()
                return
            super().mouseMoveEvent(event)

        def mouseReleaseEvent(self, event):
            if self._pan_anchor is not None:
                self._pan_anchor = None
                event.accept()
                return
            super().mouseReleaseEvent(event)


class TrajectoryPlannerWindow(QMainWindow if QT_AVAILABLE else object):
    def __init__(self, parent=None):
        if not QT_AVAILABLE:
            raise RuntimeError(f"PyQt5 is required for GUI mode: {QT_IMPORT_ERROR}")
        if not OCC_AVAILABLE:
            raise RuntimeError(f"pythonocc/OCC is required for GUI mode: {OCC_IMPORT_ERROR}")
        super().__init__(None)
        self.source_window = parent
        self.setWindowTitle("Trajectory Planner 1.A")
        self.resize(1380, 840)
        self.viewer = PanablePlannerViewer(self)
        self.trajectories = []
        self.programs = []
        self._displayed = []

        self.weld_list = QListWidget()
        self.notes_edit = QTextEdit()
        self.notes_edit.setReadOnly(True)

        self.profile_combo = QComboBox()
        self.profile_combo.addItems(["trapezoidal", "s_curve"])
        self.tcp_combo = QComboBox()
        self.tcp_combo.addItems(["bisector_normal", "world_z_up", "custom"])

        self.speed_spin = self._spin(0.1, 10000.0, 20.0, " mm/s")
        self.accel_spin = self._spin(0.1, 100000.0, 200.0, " mm/s2")
        self.jerk_spin = self._spin(0.1, 1000000.0, 2000.0, " mm/s3")
        self.flip_z_check = QCheckBox("Flip torch Z")

        self.approach_dist_spin = self._spin(0.0, 1000.0, 20.0, " mm")
        self.approach_speed_spin = self._spin(0.1, 10000.0, 50.0, " mm/s")
        self.retract_dist_spin = self._spin(0.0, 1000.0, 20.0, " mm")
        self.retract_speed_spin = self._spin(0.1, 10000.0, 50.0, " mm/s")

        self.laser_spin = self._spin(0.0, 100000.0, 1500.0, " W")
        self.feed_spin = self._spin(0.0, 1000.0, 5.0, " m/min")
        self.gas_spin = self._spin(0.0, 1000.0, 15.0, " L/min")
        self.focus_spin = self._spin(-100.0, 100.0, 0.0, " mm")
        self.ramp_up_spin = self._spin(0.0, 100.0, 0.2, " s")
        self.ramp_down_spin = self._spin(0.0, 100.0, 0.2, " s")
        self.chk_frames = QCheckBox("TCP frames")
        self.chk_frames.setChecked(True)

        self._create_menu()
        self._create_layout()
        self._update_status_bar()

    def _spin(self, lo, hi, value, suffix):
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(3)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    def _create_menu(self):
        file_menu = self.menuBar().addMenu("File")
        open_action = QAction("Open Trajectory JSON", self)
        open_action.setShortcut(QKeySequence("Ctrl+O"))
        open_action.triggered.connect(self.open_trajectory_json)
        file_menu.addAction(open_action)

        save_action = QAction("Save Program JSON", self)
        save_action.setShortcut(QKeySequence("Ctrl+S"))
        save_action.triggered.connect(self.save_program_dialog)
        file_menu.addAction(save_action)

        export_action = QAction("Export for Robot", self)
        export_action.setShortcut(QKeySequence("Ctrl+E"))
        export_action.triggered.connect(self.export_robot_dialog)
        file_menu.addAction(export_action)

        workspace_menu = self.menuBar().addMenu("Workspace")
        back_action = QAction("Back to Previous Window", self)
        back_action.setShortcut(QKeySequence("Ctrl+B"))
        back_action.triggered.connect(self.back_to_source)
        back_action.setEnabled(self.source_window is not None)
        workspace_menu.addAction(back_action)

        view_menu = self.menuBar().addMenu("View")
        fit_action = QAction("Fit All", self)
        fit_action.setShortcut(QKeySequence("F"))
        fit_action.triggered.connect(self.fit_all)
        view_menu.addAction(fit_action)

    def _create_layout(self):
        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        if self.source_window is not None:
            back_btn = QPushButton("Back to Previous Window")
            back_btn.clicked.connect(self.back_to_source)
            panel_layout.addWidget(back_btn)

        panel_layout.addWidget(self._navigate_box())
        panel_layout.addWidget(QLabel("Welds"))
        panel_layout.addWidget(self.weld_list)
        panel_layout.addWidget(self._motion_box())
        panel_layout.addWidget(self._tcp_box())
        panel_layout.addWidget(self._approach_box())
        panel_layout.addWidget(self._process_box())

        apply_selected = QPushButton("Apply to selected weld")
        apply_selected.clicked.connect(self.apply_selected)
        apply_all = QPushButton("Apply to all welds")
        apply_all.clicked.connect(self.apply_all)
        panel_layout.addWidget(apply_selected)
        panel_layout.addWidget(apply_all)

        panel_layout.addWidget(QLabel("Notes"))
        panel_layout.addWidget(self.notes_edit, stretch=2)
        save_btn = QPushButton("Save Program JSON")
        save_btn.clicked.connect(self.save_program_dialog)
        export_btn = QPushButton("Export for Robot")
        export_btn.clicked.connect(self.export_robot_dialog)
        load_robodk_btn = QPushButton("Load to RoboDK")
        load_robodk_btn.clicked.connect(self.load_to_robodk_dialog)
        panel_layout.addWidget(save_btn)
        panel_layout.addWidget(export_btn)
        panel_layout.addWidget(load_robodk_btn)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        scroll.setMinimumWidth(420)

        container = QWidget()
        layout = QHBoxLayout(container)
        layout.addWidget(self.viewer, stretch=5)
        layout.addWidget(scroll, stretch=2)
        self.setCentralWidget(container)

    def _motion_box(self):
        box = QGroupBox("Motion")
        layout = QVBoxLayout(box)
        layout.addWidget(QLabel("Profile"))
        layout.addWidget(self.profile_combo)
        layout.addWidget(QLabel("Target speed"))
        layout.addWidget(self.speed_spin)
        layout.addWidget(QLabel("Max acceleration"))
        layout.addWidget(self.accel_spin)
        layout.addWidget(QLabel("Max jerk"))
        layout.addWidget(self.jerk_spin)
        return box

    def _navigate_box(self):
        box = QGroupBox("Navigate")
        layout = QVBoxLayout(box)
        top = QHBoxLayout()
        mid = QHBoxLayout()
        bottom = QHBoxLayout()

        btn_up = QPushButton("Up")
        btn_up.clicked.connect(lambda: self.pan_view(0, 80))
        top.addStretch()
        top.addWidget(btn_up)
        top.addStretch()

        btn_left = QPushButton("Left")
        btn_fit = QPushButton("Fit")
        btn_right = QPushButton("Right")
        btn_left.clicked.connect(lambda: self.pan_view(-80, 0))
        btn_fit.clicked.connect(self.fit_all)
        btn_right.clicked.connect(lambda: self.pan_view(80, 0))
        mid.addWidget(btn_left)
        mid.addWidget(btn_fit)
        mid.addWidget(btn_right)

        btn_down = QPushButton("Down")
        btn_down.clicked.connect(lambda: self.pan_view(0, -80))
        bottom.addStretch()
        bottom.addWidget(btn_down)
        bottom.addStretch()

        layout.addLayout(top)
        layout.addLayout(mid)
        layout.addLayout(bottom)
        return box

    def _tcp_box(self):
        box = QGroupBox("TCP")
        layout = QVBoxLayout(box)
        layout.addWidget(QLabel("Strategy"))
        layout.addWidget(self.tcp_combo)
        layout.addWidget(self.flip_z_check)
        layout.addWidget(self.chk_frames)
        return box

    def _approach_box(self):
        box = QGroupBox("Approach / Retract")
        layout = QVBoxLayout(box)
        layout.addWidget(QLabel("Approach distance"))
        layout.addWidget(self.approach_dist_spin)
        layout.addWidget(QLabel("Approach speed"))
        layout.addWidget(self.approach_speed_spin)
        layout.addWidget(QLabel("Retract distance"))
        layout.addWidget(self.retract_dist_spin)
        layout.addWidget(QLabel("Retract speed"))
        layout.addWidget(self.retract_speed_spin)
        return box

    def _process_box(self):
        box = QGroupBox("Process")
        layout = QVBoxLayout(box)
        layout.addWidget(QLabel("Laser power"))
        layout.addWidget(self.laser_spin)
        layout.addWidget(QLabel("Wire feed"))
        layout.addWidget(self.feed_spin)
        layout.addWidget(QLabel("Gas flow"))
        layout.addWidget(self.gas_spin)
        layout.addWidget(QLabel("Focus offset"))
        layout.addWidget(self.focus_spin)
        layout.addWidget(QLabel("Ramp up"))
        layout.addWidget(self.ramp_up_spin)
        layout.addWidget(QLabel("Ramp down"))
        layout.addWidget(self.ramp_down_spin)
        return box

    def _configs_from_ui(self):
        motion = MotionConfig(
            target_speed_mm_s=self.speed_spin.value(),
            acceleration_max_mm_s2=self.accel_spin.value(),
            jerk_max_mm_s3=self.jerk_spin.value(),
            profile=self.profile_combo.currentText(),
        )
        tcp = TCPConfig(
            strategy=self.tcp_combo.currentText(),
            flip_torch_z=self.flip_z_check.isChecked(),
        )
        approach = ApproachRetractConfig(
            distance_mm=self.approach_dist_spin.value(),
            speed_mm_s=self.approach_speed_spin.value(),
        )
        retract = ApproachRetractConfig(
            distance_mm=self.retract_dist_spin.value(),
            speed_mm_s=self.retract_speed_spin.value(),
        )
        process = ProcessConfig(
            laser_power_w=self.laser_spin.value(),
            wire_feed_rate=self.feed_spin.value(),
            shielding_gas_flow=self.gas_spin.value(),
            focus_offset_mm=self.focus_spin.value(),
            ramp_up_time_s=self.ramp_up_spin.value(),
            ramp_down_time_s=self.ramp_down_spin.value(),
        )
        return motion, tcp, approach, retract, process

    def load_trajectories(self, trajectories):
        self.trajectories = trajectories or []
        self.programs = [None for _ in self.trajectories]
        self._refresh_weld_list()
        self.refresh_viewer()
        self._update_status_bar()

    def _refresh_weld_list(self):
        self.weld_list.clear()
        for i, traj in enumerate(self.trajectories):
            planned = self.programs[i] is not None if i < len(self.programs) else False
            status = "planned" if planned else "not planned"
            self.weld_list.addItem(
                QListWidgetItem(f"Weld {traj.weld_id}: {len(traj.points)} pts ({status})")
            )

    def apply_selected(self):
        row = self.weld_list.currentRow()
        if row < 0 and self.trajectories:
            row = 0
        if row < 0:
            return
        self._plan_index(row)

    def apply_all(self):
        for i in range(len(self.trajectories)):
            self._plan_index(i)

    def _plan_index(self, i):
        motion, tcp, approach, retract, process = self._configs_from_ui()
        self.programs[i] = plan_weld(
            self.trajectories[i],
            motion=motion,
            tcp=tcp,
            approach=approach,
            retract=retract,
            process=process,
        )
        self._refresh_weld_list()
        self._refresh_notes()
        self.refresh_viewer()
        self._update_status_bar()

    def _refresh_notes(self):
        lines = []
        for p in self._planned_programs():
            lines.append(f"Weld {p.weld_id}: {p.total_duration_s:.3f}s, {len(p.setpoints)} setpoints")
            for note in p.notes:
                lines.append(f"  - {note}")
        self.notes_edit.setPlainText("\n".join(lines))

    def _planned_programs(self):
        return [p for p in self.programs if p is not None]

    def open_trajectory_json(self):
        if load_trajectory_json is None:
            QMessageBox.critical(self, "Import Failed", "trajectory_mapping_1A could not be imported.")
            return
        path, _ = QFileDialog.getOpenFileName(self, "Open Trajectory JSON", "", "JSON files (*.json)")
        if path:
            self.load_trajectories(load_trajectory_json(path))

    def save_program_dialog(self):
        programs = self._planned_programs()
        if not programs:
            QMessageBox.information(self, "No Program", "No planned weld programs to save.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save Program JSON", "", "JSON files (*.json)")
        if path:
            if not path.lower().endswith(".json"):
                path += ".json"
            save_program_json(programs, path)

    def export_robot_dialog(self):
        programs = self._planned_programs()
        if not programs:
            QMessageBox.information(self, "No Program", "No planned weld programs to export.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export for Robot", "weld_program_robot.json", "JSON files (*.json)")
        if path:
            if not path.lower().endswith(".json"):
                path += ".json"
            save_program_json(programs, path)

    def load_to_robodk_dialog(self):
        # Deferred import so the planner still runs without RoboDK installed.
        try:
            from robodk_bridge_1A import load_to_robodk, RoboDKBridgeError
        except ImportError as exc:
            QMessageBox.critical(
                self, "Module Missing",
                f"Cannot import robodk_bridge_1A: {exc}",
            )
            return
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Welding Program JSON into RoboDK",
            "", "JSON files (*.json)",
        )
        if not path:
            return
        try:
            progs = load_to_robodk(path)
        except RoboDKBridgeError as exc:
            QMessageBox.critical(self, "RoboDK Bridge Failed", str(exc))
            return
        except Exception as exc:
            QMessageBox.critical(
                self, "RoboDK Bridge Failed",
                f"Unexpected error: {exc}",
            )
            return
        names = []
        for p in progs:
            try:
                names.append(p.Name())
            except Exception:
                names.append("<program>")
        QMessageBox.information(
            self, "Loaded to RoboDK",
            "Created {} program(s):\n  - {}".format(len(progs), "\n  - ".join(names)),
        )

    def _clear_viewer(self):
        try:
            self.viewer._display.EraseAll()
        except Exception:
            pass

    def _display_shape(self, shape, color):
        try:
            self.viewer._display.DisplayShape(shape, color=color, update=False)
        except Exception:
            pass

    def refresh_viewer(self):
        self._clear_viewer()
        programs = self._planned_programs()
        if not programs:
            for traj in self.trajectories:
                for p in traj.points:
                    self._draw_point(p.position, Quantity_Color(0.1, 0.5, 0.9, Quantity_TOC_RGB))
        else:
            max_v = max((sp.velocity for p in programs for sp in p.setpoints), default=1.0)
            for program in programs:
                for i, sp in enumerate(program.setpoints):
                    rgb = (0.55, 0.55, 0.55) if sp.phase != "weld" else _velocity_color_tuple(sp.velocity, max_v)
                    self._draw_point(sp.pose.position, Quantity_Color(*rgb, Quantity_TOC_RGB))
                    if self.chk_frames.isChecked() and sp.phase == "weld" and i % 5 == 0:
                        self._draw_frame(sp)
        self.fit_all()

    def _draw_point(self, position, color):
        p = gp_Pnt(*position)
        self._display_shape(BRepBuilderAPI_MakeVertex(p).Vertex(), color)

    def _draw_axis(self, origin, direction, length, color):
        start = gp_Pnt(*origin)
        end = gp_Pnt(
            origin[0] + direction[0] * length,
            origin[1] + direction[1] * length,
            origin[2] + direction[2] * length,
        )
        self._display_shape(BRepBuilderAPI_MakeEdge(start, end).Edge(), color)

    def _draw_frame(self, sp):
        R = sp.pose.rotation_matrix
        origin = sp.pose.position
        x = (R[0][0], R[1][0], R[2][0])
        y = (R[0][1], R[1][1], R[2][1])
        z = (R[0][2], R[1][2], R[2][2])
        self._draw_axis(origin, x, 5.0, Quantity_Color(1.0, 0.05, 0.05, Quantity_TOC_RGB))
        self._draw_axis(origin, y, 5.0, Quantity_Color(0.05, 0.85, 0.1, Quantity_TOC_RGB))
        self._draw_axis(origin, z, 5.0, Quantity_Color(0.1, 0.25, 1.0, Quantity_TOC_RGB))

    def fit_all(self):
        try:
            self.viewer._display.FitAll()
            self.viewer._display.Repaint()
        except Exception:
            pass

    def pan_view(self, dx, dy):
        try:
            self.viewer._display.Pan(dx, dy)
            self.viewer._display.Repaint()
        except Exception as exc:
            self.statusBar().showMessage(f"Pan failed: {exc}")

    def back_to_source(self):
        if self.source_window is not None:
            self.source_window.showNormal()
            self.source_window.raise_()
            self.source_window.activateWindow()

    def _update_status_bar(self):
        planned = len(self._planned_programs())
        total_time = sum(p.total_duration_s for p in self._planned_programs())
        self.statusBar().showMessage(
            f"Welds: {len(self.trajectories)} | Planned: {planned}/{len(self.trajectories)} | "
            f"Total time: {total_time:.3f}s"
        )


def launch_with_trajectories(trajectories, parent=None, motion=None, tcp=None, approach=None, retract=None, process=None):
    if not QT_AVAILABLE:
        raise RuntimeError(f"PyQt5 is required for GUI mode: {QT_IMPORT_ERROR}")
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    window = TrajectoryPlannerWindow(parent=parent)
    window.load_trajectories(trajectories)
    if any(x is not None for x in (motion, tcp, approach, retract, process)):
        window.programs = plan_all_welds(
            trajectories,
            motion=motion,
            tcp=tcp,
            approach=approach,
            retract=retract,
            process=process,
        )
        window._refresh_weld_list()
        window._refresh_notes()
        window.refresh_viewer()
    window.show()
    return window


def _run_cli(args):
    if load_trajectory_json is None:
        raise RuntimeError("trajectory_mapping_1A could not be imported")
    trajectories = load_trajectory_json(args.input)
    motion = MotionConfig(
        target_speed_mm_s=args.speed,
        acceleration_max_mm_s2=args.accel,
        jerk_max_mm_s3=args.jerk,
        profile=args.profile,
    )
    process = ProcessConfig(
        laser_power_w=args.laser,
        wire_feed_rate=args.feed,
        shielding_gas_flow=args.gas,
        focus_offset_mm=args.focus,
        ramp_up_time_s=args.ramp_up,
        ramp_down_time_s=args.ramp_down,
    )
    programs = plan_all_welds(trajectories, motion=motion, process=process)
    save_program_json(programs, args.output)
    print(f"Planned {len(programs)} weld program(s); wrote {args.output}")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Trajectory Planner 1.A")
    parser.add_argument("input_pos", nargs="?", help="Input trajectory JSON for GUI mode")
    parser.add_argument("--input", help="Input trajectory JSON")
    parser.add_argument("--output", help="Output weld program JSON")
    parser.add_argument("--gui", action="store_true", help="Open GUI")
    parser.add_argument("--speed", type=float, default=20.0)
    parser.add_argument("--accel", type=float, default=200.0)
    parser.add_argument("--jerk", type=float, default=2000.0)
    parser.add_argument("--profile", default="trapezoidal", choices=["trapezoidal", "s_curve"])
    parser.add_argument("--laser", type=float, default=1500.0)
    parser.add_argument("--feed", type=float, default=5.0)
    parser.add_argument("--gas", type=float, default=15.0)
    parser.add_argument("--focus", type=float, default=0.0)
    parser.add_argument("--ramp-up", type=float, default=0.2)
    parser.add_argument("--ramp-down", type=float, default=0.2)
    args = parser.parse_args(argv)

    input_path = args.input or args.input_pos
    if args.gui or not args.output:
        if not QT_AVAILABLE:
            raise RuntimeError(f"PyQt5 is required for GUI mode: {QT_IMPORT_ERROR}")
        app = QApplication.instance() or QApplication(sys.argv)
        window = TrajectoryPlannerWindow()
        if input_path:
            if load_trajectory_json is None:
                raise RuntimeError("trajectory_mapping_1A could not be imported")
            window.load_trajectories(load_trajectory_json(input_path))
        window.show()
        return app.exec_()

    if not input_path:
        raise RuntimeError("--input is required in headless mode")
    return _run_cli(argparse.Namespace(**vars(args), input=input_path))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
