import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass, field

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
    """Posed TCP setpoint derived from one mapped weld point."""
    index: int
    arc_length: float
    phase: str
    pose: TCPPose
    notes: list = field(default_factory=list)

    def to_dict(self):
        return {
            "index": self.index,
            "arc_length": self.arc_length,
            "phase": self.phase,
            "pose": {
                "position": list(self.pose.position),
                "rotation_matrix": self.pose.rotation_matrix,
                "quaternion": list(self.pose.quaternion),
                "euler_zyx_deg": list(self.pose.euler_zyx_deg),
            },
            "notes": list(self.notes),
        }


@dataclass
class TCPConfig:
    strategy: str = "bisector_normal"
    flip_torch_z: bool = False
    custom_world_z: tuple = (0.0, 0.0, 1.0)


@dataclass
class WeldProgram:
    weld_id: int
    weld_name: str
    tcp: TCPConfig
    setpoints: list = field(default_factory=list)
    notes: list = field(default_factory=list)

    def to_dict(self):
        return {
            "weld_id": self.weld_id,
            "weld_name": self.weld_name,
            "tcp": asdict(self.tcp),
            "setpoints": [s.to_dict() for s in self.setpoints],
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


def _make_setpoint(index, arc_length, pose, notes=None):
    return TrajectorySetpoint(
        index=index,
        arc_length=arc_length,
        phase="weld",
        pose=pose,
        notes=list(notes or []),
    )


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


def plan_weld(trajectory, tcp=None):
    tcp = tcp or TCPConfig()
    program = WeldProgram(
        weld_id=trajectory.weld_id,
        weld_name=trajectory.weld_name,
        tcp=tcp,
    )

    points = list(trajectory.points)
    if not points:
        program.notes.append("trajectory has no points")
        return program

    for i, point in enumerate(points):
        pose, pose_notes = compute_tcp_frame(point.position, point.tangent, point.normal, tcp)
        program.setpoints.append(_make_setpoint(
            i,
            point.arc_length,
            pose,
            notes=pose_notes,
        ))

    validate_program(program)
    return program


def plan_all_welds(trajectories, tcp=None):
    return [plan_weld(t, tcp=tcp) for t in trajectories]


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
        arc_length=data["arc_length"],
        phase=data["phase"],
        pose=_pose_from_dict(data["pose"]),
        notes=data.get("notes", []),
    )


def _program_from_dict(data):
    return WeldProgram(
        weld_id=data["weld_id"],
        weld_name=data["weld_name"],
        tcp=TCPConfig(**data.get("tcp", {})),
        setpoints=[_setpoint_from_dict(s) for s in data.get("setpoints", [])],
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


def save_program_csv(programs, filepath):
    with open(filepath, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        for program in programs:
            for sp in program.setpoints:
                if sp.phase != "weld":
                    continue
                R = sp.pose.rotation_matrix
                writer.writerow([
                    sp.pose.position[0],
                    sp.pose.position[1],
                    sp.pose.position[2],
                    R[0][2],
                    R[1][2],
                    R[2][2],
                ])


def load_program_json(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        payload = json.load(f)
    return [_program_from_dict(p) for p in payload.get("programs", [])]


try:
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QKeySequence
    from PyQt5.QtWidgets import (
        QAction,
        QApplication,
        QCheckBox,
        QComboBox,
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

        self.tcp_combo = QComboBox()
        self.tcp_combo.addItems(["bisector_normal", "world_z_up", "custom"])
        self.flip_z_check = QCheckBox("Flip torch Z")
        self.chk_frames = QCheckBox("TCP frames")
        self.chk_frames.setChecked(True)

        self._create_menu()
        self._create_layout()
        self._update_status_bar()

    def _create_menu(self):
        file_menu = self.menuBar().addMenu("File")
        open_action = QAction("Open Trajectory JSON", self)
        open_action.setShortcut(QKeySequence("Ctrl+O"))
        open_action.triggered.connect(self.open_trajectory_json)
        file_menu.addAction(open_action)

        save_action = QAction("Save JSON + CSV", self)
        save_action.setShortcut(QKeySequence("Ctrl+S"))
        save_action.triggered.connect(self.save_program_dialog)
        file_menu.addAction(save_action)

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
        panel_layout.addWidget(self._tcp_box())

        apply_selected = QPushButton("Apply to selected weld")
        apply_selected.clicked.connect(self.apply_selected)
        apply_all = QPushButton("Apply to all welds")
        apply_all.clicked.connect(self.apply_all)
        panel_layout.addWidget(apply_selected)
        panel_layout.addWidget(apply_all)

        panel_layout.addWidget(QLabel("Notes"))
        panel_layout.addWidget(self.notes_edit, stretch=2)
        save_btn = QPushButton("Save JSON + CSV")
        save_btn.clicked.connect(self.save_program_dialog)
        panel_layout.addWidget(save_btn)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(panel)
        scroll.setMinimumWidth(420)

        container = QWidget()
        layout = QHBoxLayout(container)
        layout.addWidget(self.viewer, stretch=5)
        layout.addWidget(scroll, stretch=2)
        self.setCentralWidget(container)

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

    def _configs_from_ui(self):
        return TCPConfig(
            strategy=self.tcp_combo.currentText(),
            flip_torch_z=self.flip_z_check.isChecked(),
        )

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
        self.programs[i] = plan_weld(self.trajectories[i], tcp=self._configs_from_ui())
        self._refresh_weld_list()
        self._refresh_notes()
        self.refresh_viewer()
        self._update_status_bar()

    def _refresh_notes(self):
        lines = []
        for p in self._planned_programs():
            lines.append(f"Weld {p.weld_id}: {len(p.setpoints)} setpoints")
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
        path, _ = QFileDialog.getSaveFileName(self, "Save JSON + CSV", "", "JSON files (*.json)")
        if path:
            if not path.lower().endswith(".json"):
                path += ".json"
            csv_path = path[:-5] + ".csv"
            save_program_json(programs, path)
            save_program_csv(programs, csv_path)
            QMessageBox.information(self, "Saved", f"Saved:\n{path}\n{csv_path}")

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
            for program in programs:
                for i, sp in enumerate(program.setpoints):
                    self._draw_point(sp.pose.position, Quantity_Color(0.1, 0.5, 0.9, Quantity_TOC_RGB))
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
            self.hide()

    def _update_status_bar(self):
        planned = len(self._planned_programs())
        self.statusBar().showMessage(
            f"Welds: {len(self.trajectories)} | Planned: {planned}/{len(self.trajectories)}"
        )


def launch_with_trajectories(trajectories, parent=None, tcp=None):
    if not QT_AVAILABLE:
        raise RuntimeError(f"PyQt5 is required for GUI mode: {QT_IMPORT_ERROR}")
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    window = TrajectoryPlannerWindow(parent=parent)
    window.load_trajectories(trajectories)
    if tcp is not None:
        window.programs = plan_all_welds(trajectories, tcp=tcp)
        window._refresh_weld_list()
        window._refresh_notes()
        window.refresh_viewer()
    window.show()
    return window


def _run_cli(args):
    if load_trajectory_json is None:
        raise RuntimeError("trajectory_mapping_1A could not be imported")
    trajectories = load_trajectory_json(args.input)
    programs = plan_all_welds(trajectories)
    save_program_json(programs, args.output)
    csv_path = args.output[:-5] + ".csv" if args.output.lower().endswith(".json") else args.output + ".csv"
    save_program_csv(programs, csv_path)
    print(f"Planned {len(programs)} weld program(s); wrote {args.output} and {csv_path}")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Trajectory Planner 1.A")
    parser.add_argument("input_pos", nargs="?", help="Input trajectory JSON for GUI mode")
    parser.add_argument("--input", help="Input trajectory JSON")
    parser.add_argument("--output", help="Output weld program JSON")
    parser.add_argument("--gui", action="store_true", help="Open GUI")
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

