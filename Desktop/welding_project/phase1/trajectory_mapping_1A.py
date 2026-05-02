import argparse
import json
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import Optional


@dataclass
class WeldPoint:
    """Single point on a weld trajectory, ordered by distance from the weld start."""
    index: int
    arc_length: float
    position: tuple
    tangent: tuple
    normal: tuple

    def to_dict(self):
        return {
            "index": self.index,
            "arc_length": self.arc_length,
            "position": list(self.position),
            "tangent": list(self.tangent),
            "normal": list(self.normal),
        }


@dataclass
class WeldTrajectory:
    """Trajectory data for one weld: discretized points plus metadata."""
    weld_id: int
    weld_name: str
    segments_meta: list
    points: list
    total_length: float
    discretization_step_mm: float
    notes: list = field(default_factory=list)

    def to_dict(self):
        return {
            "weld_id": self.weld_id,
            "weld_name": self.weld_name,
            "segments_meta": self.segments_meta,
            "points": [p.to_dict() for p in self.points],
            "total_length": self.total_length,
            "discretization_step_mm": self.discretization_step_mm,
            "notes": self.notes,
        }


# OCC imports are kept optional at module import time so JSON-only CLI operations
# can still run in environments where pythonocc is not available.
try:
    from OCC.Display.backend import load_backend
    load_backend("pyqt5")
    from OCC.Display.qtDisplay import qtViewer3d

    from OCC.Core.AIS import AIS_Shape
    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.BRepAdaptor import BRepAdaptor_CompCurve, BRepAdaptor_Surface
    from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeVertex, BRepBuilderAPI_MakeWire
    from OCC.Core.BRepGProp import brepgprop
    from OCC.Core.BRepLProp import BRepLProp_SLProps
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.GCPnts import GCPnts_AbscissaPoint, GCPnts_UniformAbscissa
    from OCC.Core.GProp import GProp_GProps
    from OCC.Core.gp import gp_Pnt, gp_Vec
    from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
    from OCC.Core.ShapeAnalysis import ShapeAnalysis_Surface
    from OCC.Core.STEPControl import STEPControl_Reader
    from OCC.Core.TopAbs import TopAbs_REVERSED
    from OCC.Extend.TopologyUtils import TopologyExplorer

    OCC_AVAILABLE = True
except Exception as exc:
    OCC_IMPORT_ERROR = exc
    OCC_AVAILABLE = False


def _require_occ():
    if not OCC_AVAILABLE:
        raise RuntimeError(f"pythonocc/OCC is required for trajectory computation: {OCC_IMPORT_ERROR}")


def _normalize(vec, eps=1e-9):
    mag = (vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]) ** 0.5
    if mag < eps:
        return (0.0, 0.0, 0.0), False
    return (vec[0] / mag, vec[1] / mag, vec[2] / mag), True


def _shape_length(shape):
    _require_occ()
    props = GProp_GProps()
    brepgprop.LinearProperties(shape, props)
    return props.Mass()


def _point_tuple(pnt):
    return (pnt.X(), pnt.Y(), pnt.Z())


def _point_distance(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def _edge_endpoints(edge):
    try:
        curve_data = BRep_Tool.Curve(edge)
        if curve_data is None or len(curve_data) < 3:
            return None
        curve, u_min, u_max = curve_data[0], curve_data[-2], curve_data[-1]
        if curve is None:
            return None
        return _point_tuple(curve.Value(u_min)), _point_tuple(curve.Value(u_max))
    except Exception:
        return None


def _read_step_shape(step_file_path):
    _require_occ()
    reader = STEPControl_Reader()
    status = reader.ReadFile(step_file_path)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"Could not read STEP file: {step_file_path}")
    reader.TransferRoots()
    return reader.OneShape()


def _resolve_step_path(step_file_path, json_path=None):
    if not step_file_path:
        return None
    candidates = [step_file_path]
    if json_path and not os.path.isabs(step_file_path):
        candidates.append(os.path.join(os.path.dirname(json_path), step_file_path))
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            return os.path.abspath(candidate)
    return step_file_path


def _find_matching_edge(edges, start_xyz, end_xyz, tolerance=0.25):
    best = None
    best_score = None
    best_reversed = False

    for edge in edges:
        endpoints = _edge_endpoints(edge)
        if endpoints is None:
            continue
        a, b = endpoints
        forward = _point_distance(a, start_xyz) + _point_distance(b, end_xyz)
        reverse = _point_distance(a, end_xyz) + _point_distance(b, start_xyz)
        score = min(forward, reverse)
        if best_score is None or score < best_score:
            best = edge
            best_score = score
            best_reversed = reverse < forward

    if best is None or best_score is None or best_score > tolerance * 2.0:
        return None, False, best_score
    return best, best_reversed, best_score


def _make_wire_from_edge(edge, reversed_edge=False):
    if reversed_edge:
        reversed_shape = edge.Reversed()
        try:
            from OCC.Core.TopoDS import topods
            edge = topods.Edge(reversed_shape)
        except Exception:
            edge = reversed_shape
    return BRepBuilderAPI_MakeWire(edge).Wire()


def reconstruct_welds_from_metadata_json(payload, json_path=None, endpoint_tolerance=0.25):
    """
    Best-effort standalone bridge for 5.K JSON files.
    Reopens the STEP file and matches exported segment start/end points to STEP edges.
    Face references are not serialized in 5.K JSON, so reconstructed segments use default normals.
    """
    _require_occ()
    step_path = _resolve_step_path(payload.get("step_file"), json_path=json_path)
    if not step_path or not os.path.exists(step_path):
        raise RuntimeError(
            "This JSON references no readable STEP file. Reopen/export from 5.K, "
            "or put the STEP file at the path stored in the JSON."
        )

    root = _read_step_shape(step_path)
    edges = list(TopologyExplorer(root).edges())
    if not edges:
        raise RuntimeError(f"No edges found in STEP file: {step_path}")

    reconstructed = []
    notes = []
    for weld in payload.get("welds", []):
        new_weld = {
            "id": int(weld.get("id", len(reconstructed) + 1)),
            "name": weld.get("name", f"Weld {len(reconstructed) + 1}"),
            "context": weld.get("context", "metadata_reconstructed"),
            "segments": [],
        }
        for seg in weld.get("segments", []):
            start = seg.get("start_point")
            end = seg.get("end_point")
            if not start or not end:
                notes.append(f"Weld {new_weld['id']} segment {seg.get('index', '?')}: missing endpoints")
                continue

            edge, reversed_edge, score = _find_matching_edge(
                edges, tuple(start), tuple(end), tolerance=endpoint_tolerance
            )
            if edge is None:
                notes.append(
                    f"Weld {new_weld['id']} segment {seg.get('index', '?')}: "
                    f"no STEP edge matched endpoints (best error={score})"
                )
                continue

            wire = _make_wire_from_edge(edge, reversed_edge=reversed_edge)
            new_seg = dict(seg)
            new_seg.update({
                "wire": wire,
                "face_a": None,
                "face_b": None,
                "method": f"{seg.get('method', 'metadata')} / STEP endpoint match",
                "reconstructed_from_json": True,
            })
            new_weld["segments"].append(new_seg)
        reconstructed.append(new_weld)

    return reconstructed, step_path, notes


def wire_to_compcurve(wire):
    """
    TopoDS_Wire -> BRepAdaptor_CompCurve.
    All wire edges are mapped into a single parameter space.
    Returns: (compcurve, u_min, u_max)
    """
    _require_occ()
    cc = BRepAdaptor_CompCurve(wire)
    return cc, cc.FirstParameter(), cc.LastParameter()


def discretize_uniform(compcurve, u_min, u_max, step_mm):
    """
    Sample a wire at roughly uniform arc-length intervals.
    Returns list of (u_param, arc_length) tuples sorted from start to end.
    """
    _require_occ()
    if step_mm <= 0:
        raise ValueError("step_mm must be greater than zero")

    total_len = GCPnts_AbscissaPoint.Length(compcurve, u_min, u_max)
    if total_len <= 1e-9:
        return [(u_min, 0.0)]

    if total_len < step_mm:
        return [(u_min, 0.0), (u_max, total_len)]

    abs_calc = GCPnts_UniformAbscissa(compcurve, step_mm, u_min, u_max)
    if not abs_calc.IsDone():
        return [(u_min, 0.0), (u_max, total_len)]

    samples = []
    for i in range(1, abs_calc.NbPoints() + 1):
        u = abs_calc.Parameter(i)
        arc = GCPnts_AbscissaPoint.Length(compcurve, u_min, u)
        samples.append((u, arc))

    if samples and abs(samples[-1][1] - total_len) > max(1e-6, step_mm * 0.25):
        samples.append((u_max, total_len))
    return samples


def point_and_tangent(compcurve, u):
    """
    Evaluate point and unit tangent at parameter u.
    Returns: ((x, y, z), (tx, ty, tz), singularity_flag)
    """
    _require_occ()
    point = gp_Pnt()
    tangent = gp_Vec()
    compcurve.D1(u, point, tangent)

    pos = (point.X(), point.Y(), point.Z())
    t, ok = _normalize((tangent.X(), tangent.Y(), tangent.Z()))
    if not ok:
        return pos, (0.0, 0.0, 0.0), True
    return pos, t, False


def point_on_face_normal(face, point_xyz, default=(0.0, 0.0, 1.0)):
    """
    Project a 3D point into face UV space and compute the local surface normal.
    Returns: ((nx, ny, nz), is_valid)
    """
    _require_occ()
    try:
        pnt = gp_Pnt(*point_xyz)
        surf_adaptor = BRepAdaptor_Surface(face)
        surface = BRep_Tool.Surface(face)
        sa = ShapeAnalysis_Surface(surface)
        uv = sa.ValueOfUV(pnt, 1e-3)
        u, v = uv.X(), uv.Y()

        props = BRepLProp_SLProps(surf_adaptor, u, v, 1, 1e-6)
        if not props.IsNormalDefined():
            return default, False

        normal = props.Normal()
        if face.Orientation() == TopAbs_REVERSED:
            normal.Reverse()

        n, ok = _normalize((normal.X(), normal.Y(), normal.Z()))
        return (n if ok else default), ok
    except Exception:
        return default, False


def bisector_normal(face_a, face_b, point_xyz):
    """
    Bisector of two face normals at a point.
    Returns: ((nx, ny, nz), is_valid_flag, source_str)
    """
    n_a, valid_a = (None, False)
    n_b, valid_b = (None, False)

    if face_a is not None:
        n_a, valid_a = point_on_face_normal(face_a, point_xyz)
    if face_b is not None:
        n_b, valid_b = point_on_face_normal(face_b, point_xyz)

    if valid_a and valid_b:
        bisector, ok = _normalize((n_a[0] + n_b[0], n_a[1] + n_b[1], n_a[2] + n_b[2]), eps=1e-6)
        if not ok:
            return n_a, False, "anti_parallel_fallback_a"
        return bisector, True, "bisector"

    if valid_a:
        return n_a, True, "face_a_only"
    if valid_b:
        return n_b, True, "face_b_only"
    return (0.0, 0.0, 1.0), False, "default_z"


def _repair_singular_tangents(points, notes):
    for i, point in enumerate(points):
        if point.tangent != (0.0, 0.0, 0.0):
            continue

        candidates = []
        if i > 0 and points[i - 1].tangent != (0.0, 0.0, 0.0):
            candidates.append(points[i - 1].tangent)
        if i + 1 < len(points) and points[i + 1].tangent != (0.0, 0.0, 0.0):
            candidates.append(points[i + 1].tangent)

        if candidates:
            avg = (
                sum(v[0] for v in candidates),
                sum(v[1] for v in candidates),
                sum(v[2] for v in candidates),
            )
            tangent, ok = _normalize(avg)
            if ok:
                point.tangent = tangent
                notes.append(f"Point {point.index}: tangent singularity, used neighbor average")


def compute_segment_trajectory(segment_dict, step_mm, point_offset=0):
    """
    Compute WeldPoint list for one segment dictionary from 5.K in-memory data.
    Returns: (list[WeldPoint], list[str])
    """
    notes = []
    points = []

    wire = segment_dict.get("wire")
    if wire is None:
        notes.append("Segment has no wire geometry; cannot compute trajectory")
        return points, notes

    face_a = segment_dict.get("face_a")
    face_b = segment_dict.get("face_b")
    missing_faces_note_added = False

    cc, u_min, u_max = wire_to_compcurve(wire)
    samples = discretize_uniform(cc, u_min, u_max, step_mm)
    if not samples:
        notes.append("Segment had zero discretization points")
        return points, notes

    for local_i, (u, arc) in enumerate(samples):
        idx = point_offset + local_i
        pos, tangent, t_singular = point_and_tangent(cc, u)
        if t_singular:
            notes.append(f"Point {idx}: tangent singularity at u={u:.3f}")

        normal, n_valid, n_source = bisector_normal(face_a, face_b, pos)
        if not n_valid:
            if face_a is None and face_b is None:
                if not missing_faces_note_added:
                    notes.append("No face geometry available; using default Z normals")
                    missing_faces_note_added = True
            else:
                notes.append(f"Point {idx}: normal undefined ({n_source})")

        points.append(WeldPoint(
            index=idx,
            arc_length=arc,
            position=pos,
            tangent=tangent,
            normal=normal,
        ))

    _repair_singular_tangents(points, notes)
    return points, notes


def compute_weld_trajectory(weld_dict, step_mm=1.0):
    """
    Compute one WeldTrajectory from a 5.K weld dictionary.
    Segment order is preserved.
    """
    all_points = []
    all_notes = []
    cumulative_arc = 0.0
    segments_meta = []

    for seg_idx, seg in enumerate(weld_dict.get("segments", [])):
        seg_points, seg_notes = compute_segment_trajectory(
            seg, step_mm, point_offset=len(all_points)
        )

        for p in seg_points:
            p.arc_length += cumulative_arc

        if seg_points:
            cumulative_arc = seg_points[-1].arc_length

        all_points.extend(seg_points)
        all_notes.extend([f"[Segment {seg_idx + 1}] {note}" for note in seg_notes])

        length = seg.get("length")
        if length is None and seg.get("wire") is not None:
            try:
                length = _shape_length(seg["wire"])
            except Exception:
                length = 0.0

        segments_meta.append({
            "index": seg_idx,
            "type": seg.get("type", "unknown"),
            "method": seg.get("method", "unknown"),
            "length": float(length or 0.0),
            "n_points": len(seg_points),
        })

    weld_id = weld_dict.get("id", 1)
    return WeldTrajectory(
        weld_id=weld_id,
        weld_name=weld_dict.get("name", f"Weld {weld_id}"),
        segments_meta=segments_meta,
        points=all_points,
        total_length=cumulative_arc,
        discretization_step_mm=step_mm,
        notes=all_notes,
    )


def compute_trajectories(welds, step_mm=1.0):
    return [compute_weld_trajectory(weld, step_mm=step_mm) for weld in welds]


def save_trajectory_json(trajectories, filepath):
    payload = {
        "version": "trajectory_1.A",
        "n_welds": len(trajectories),
        "trajectories": [t.to_dict() for t in trajectories],
    }
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def load_trajectory_json(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        payload = json.load(f)

    out = []
    for tdict in payload.get("trajectories", []):
        points = [WeldPoint(
            index=pd["index"],
            arc_length=pd["arc_length"],
            position=tuple(pd["position"]),
            tangent=tuple(pd["tangent"]),
            normal=tuple(pd["normal"]),
        ) for pd in tdict.get("points", [])]

        out.append(WeldTrajectory(
            weld_id=tdict["weld_id"],
            weld_name=tdict["weld_name"],
            segments_meta=tdict.get("segments_meta", []),
            points=points,
            total_length=tdict.get("total_length", 0.0),
            discretization_step_mm=tdict.get("discretization_step_mm", 1.0),
            notes=tdict.get("notes", []),
        ))
    return out


def _load_json_payload(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        return json.load(f)


def _trajectory_color(index):
    palette = [
        (0.10, 0.45, 0.85),
        (0.85, 0.18, 0.12),
        (0.10, 0.60, 0.25),
        (0.65, 0.25, 0.75),
        (0.85, 0.55, 0.10),
        (0.15, 0.62, 0.62),
    ]
    rgb = palette[index % len(palette)]
    return Quantity_Color(rgb[0], rgb[1], rgb[2], Quantity_TOC_RGB)


try:
    from PyQt5.QtCore import Qt
    from PyQt5.QtGui import QKeySequence
    from PyQt5.QtWidgets import (
        QAction,
        QApplication,
        QCheckBox,
        QFileDialog,
        QHBoxLayout,
        QLabel,
        QListWidget,
        QListWidgetItem,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QDoubleSpinBox,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )

    QT_AVAILABLE = True
except Exception as exc:
    QT_IMPORT_ERROR = exc
    QT_AVAILABLE = False


class TrajectoryMappingWindow(QMainWindow if QT_AVAILABLE else object):
    """Standalone GUI wrapper around the pure trajectory computation layer."""

    def __init__(self, parent=None):
        if not QT_AVAILABLE:
            raise RuntimeError(f"PyQt5 is required for GUI mode: {QT_IMPORT_ERROR}")
        if not OCC_AVAILABLE:
            raise RuntimeError(f"pythonocc/OCC is required for GUI mode: {OCC_IMPORT_ERROR}")

        super().__init__(parent)
        self.setWindowTitle("Trajectory Mapping 1.A")
        self.resize(1380, 820)

        self.viewer = qtViewer3d(self)
        self.welds = []
        self.trajectories = []
        self.step_file_path = None
        self.loaded_json_path = None
        self._displayed = []

        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.01, 1000.0)
        self.step_spin.setDecimals(3)
        self.step_spin.setValue(1.0)
        self.step_spin.setSuffix(" mm")

        self.chk_points = QCheckBox("Points")
        self.chk_tangents = QCheckBox("Tangent vectors")
        self.chk_normals = QCheckBox("Normal vectors")
        self.chk_labels = QCheckBox("Arc length labels")
        self.chk_points.setChecked(True)
        self.chk_tangents.setChecked(True)
        self.chk_normals.setChecked(True)

        self.weld_list = QListWidget()
        self.notes_edit = QTextEdit()
        self.notes_edit.setReadOnly(True)

        self._create_menu()
        self._create_layout()
        self._connect_signals()
        self._update_status_bar()

    def _create_menu(self):
        file_menu = self.menuBar().addMenu("File")

        open_action = QAction("Open Path Extraction Output...", self)
        open_action.setShortcut(QKeySequence("Ctrl+O"))
        open_action.triggered.connect(self.open_path_output)
        file_menu.addAction(open_action)

        file_menu.addSeparator()

        save_action = QAction("Save Trajectory JSON", self)
        save_action.setShortcut(QKeySequence("Ctrl+S"))
        save_action.triggered.connect(self.save_trajectory_dialog)
        file_menu.addAction(save_action)

        export_action = QAction("Export for Phase 4", self)
        export_action.setShortcut(QKeySequence("Ctrl+E"))
        export_action.triggered.connect(self.export_phase4_dialog)
        file_menu.addAction(export_action)

    def _create_layout(self):
        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.addWidget(QLabel("Trajectory Settings"))
        panel_layout.addWidget(QLabel("Discretization step:"))
        panel_layout.addWidget(self.step_spin)

        apply_btn = QPushButton("Apply")
        apply_btn.clicked.connect(self.recompute_from_loaded_welds)
        panel_layout.addWidget(apply_btn)

        panel_layout.addWidget(QLabel("Show:"))
        panel_layout.addWidget(self.chk_points)
        panel_layout.addWidget(self.chk_tangents)
        panel_layout.addWidget(self.chk_normals)
        panel_layout.addWidget(self.chk_labels)

        panel_layout.addWidget(QLabel("Welds"))
        panel_layout.addWidget(self.weld_list, stretch=2)
        panel_layout.addWidget(QLabel("Notes (warnings):"))
        panel_layout.addWidget(self.notes_edit, stretch=3)

        save_btn = QPushButton("Save Trajectory JSON")
        save_btn.clicked.connect(self.save_trajectory_dialog)
        export_btn = QPushButton("Export for Phase 4")
        export_btn.clicked.connect(self.export_phase4_dialog)
        panel_layout.addWidget(save_btn)
        panel_layout.addWidget(export_btn)

        container = QWidget()
        layout = QHBoxLayout(container)
        layout.addWidget(self.viewer, stretch=5)
        layout.addWidget(panel, stretch=2)
        self.setCentralWidget(container)

    def _connect_signals(self):
        self.chk_points.stateChanged.connect(self.refresh_viewer)
        self.chk_tangents.stateChanged.connect(self.refresh_viewer)
        self.chk_normals.stateChanged.connect(self.refresh_viewer)
        self.chk_labels.stateChanged.connect(self.refresh_viewer)

    def open_path_output(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Path Extraction Output", "", "JSON files (*.json)"
        )
        if not path:
            return
        self.loaded_json_path = path
        try:
            payload = _load_json_payload(path)
            if "trajectories" in payload:
                self.trajectories = load_trajectory_json(path)
                self.welds = []
                self.step_file_path = payload.get("step_file")
                self._refresh_ui()
                return

            if "welds" in payload:
                welds, step_path, reconstruction_notes = reconstruct_welds_from_metadata_json(
                    payload, json_path=path
                )
                self.welds = welds
                self.step_file_path = step_path
                self.trajectories = compute_trajectories(self.welds, step_mm=self.step_spin.value())
                if reconstruction_notes:
                    for traj in self.trajectories:
                        traj.notes[:0] = reconstruction_notes
                self._refresh_ui()
                QMessageBox.information(
                    self,
                    "Loaded",
                    "JSON metadata loaded by matching segment endpoints back to STEP edges.\n\n"
                    "Note: face references are not stored in 5.K JSON, so normals use default Z. "
                    "For true bisector normals, launch this module directly from 5.K."
                )
                return

            QMessageBox.warning(self, "Unsupported JSON", "No trajectories or welds were found.")
        except Exception as exc:
            QMessageBox.critical(self, "Open Failed", str(exc))

    def load_welds(self, welds, step_file_path=None, step_mm=1.0):
        self.welds = welds or []
        self.step_file_path = step_file_path
        self.step_spin.setValue(float(step_mm))
        self.recompute_from_loaded_welds()

    def recompute_from_loaded_welds(self):
        if not self.welds:
            self._refresh_ui()
            return
        try:
            self.trajectories = compute_trajectories(self.welds, step_mm=self.step_spin.value())
            self._refresh_ui()
        except Exception as exc:
            QMessageBox.critical(self, "Trajectory Failed", str(exc))

    def _refresh_ui(self):
        self._refresh_weld_list()
        self._refresh_notes()
        self.refresh_viewer()
        self._update_status_bar()

    def _refresh_weld_list(self):
        self.weld_list.clear()
        for traj in self.trajectories:
            item = QListWidgetItem(
                f"Weld {traj.weld_id}: {len(traj.points)} pts, {traj.total_length:.2f} mm"
            )
            item.setCheckState(Qt.Checked)
            self.weld_list.addItem(item)

    def _refresh_notes(self):
        lines = []
        for traj in self.trajectories:
            for note in traj.notes:
                lines.append(f"[Weld {traj.weld_id}] {note}")
        self.notes_edit.setPlainText("\n".join(lines))

    def _clear_viewer(self):
        try:
            self.viewer._display.EraseAll()
        except Exception:
            pass
        self._displayed = []

    def _display_shape(self, shape, color=None):
        try:
            if color is None:
                ais = self.viewer._display.DisplayShape(shape, update=False)
            else:
                ais = self.viewer._display.DisplayShape(shape, color=color, update=False)
            self._displayed.append(ais)
        except Exception:
            try:
                ais = AIS_Shape(shape)
                ctx = self.viewer._display.GetContext()
                ctx.Display(ais, False)
                if color is not None:
                    ctx.SetColor(ais, color, False)
                self._displayed.append(ais)
            except Exception:
                pass

    def refresh_viewer(self):
        if not QT_AVAILABLE or not OCC_AVAILABLE:
            return
        self._clear_viewer()

        for weld_idx, weld in enumerate(self.welds):
            color = _trajectory_color(weld_idx)
            for seg in weld.get("segments", []):
                wire = seg.get("wire")
                if wire is not None:
                    self._display_shape(wire, color=color)

        tangent_color = Quantity_Color(1.0, 0.85, 0.05, Quantity_TOC_RGB)
        normal_color = Quantity_Color(0.05, 0.85, 0.20, Quantity_TOC_RGB)

        for traj_idx, traj in enumerate(self.trajectories):
            point_color = _trajectory_color(traj_idx)
            for p in traj.points:
                base = gp_Pnt(*p.position)
                if self.chk_points.isChecked():
                    self._display_shape(BRepBuilderAPI_MakeVertex(base).Vertex(), color=point_color)
                if self.chk_tangents.isChecked() and p.tangent != (0.0, 0.0, 0.0):
                    end = gp_Pnt(
                        p.position[0] + p.tangent[0] * 5.0,
                        p.position[1] + p.tangent[1] * 5.0,
                        p.position[2] + p.tangent[2] * 5.0,
                    )
                    self._display_shape(BRepBuilderAPI_MakeEdge(base, end).Edge(), color=tangent_color)
                if self.chk_normals.isChecked() and p.normal != (0.0, 0.0, 0.0):
                    end = gp_Pnt(
                        p.position[0] + p.normal[0] * 5.0,
                        p.position[1] + p.normal[1] * 5.0,
                        p.position[2] + p.normal[2] * 5.0,
                    )
                    self._display_shape(BRepBuilderAPI_MakeEdge(base, end).Edge(), color=normal_color)

        try:
            self.viewer._display.FitAll()
            self.viewer._display.Repaint()
        except Exception:
            pass

    def save_trajectory_dialog(self):
        if not self.trajectories:
            QMessageBox.information(self, "No Trajectory", "No trajectories to save.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Trajectory JSON", "", "JSON files (*.json)"
        )
        if path:
            self._save_to_path(path)

    def export_phase4_dialog(self):
        if not self.trajectories:
            QMessageBox.information(self, "No Trajectory", "No trajectories to export.")
            return
        default = "trajectory_phase4.json"
        if self.step_file_path:
            base = os.path.splitext(os.path.basename(self.step_file_path))[0]
            default = f"{base}_phase4.json"
        path, _ = QFileDialog.getSaveFileName(
            self, "Export for Phase 4", default, "JSON files (*.json)"
        )
        if path:
            if not path.lower().endswith(".json"):
                path += ".json"
            if not path.lower().endswith("_phase4.json"):
                path = path[:-5] + "_phase4.json"
            self._save_to_path(path)

    def _save_to_path(self, path):
        if not path.lower().endswith(".json"):
            path += ".json"
        try:
            save_trajectory_json(self.trajectories, path)
            QMessageBox.information(self, "Saved", f"Saved trajectory JSON:\n{path}")
        except Exception as exc:
            QMessageBox.critical(self, "Save Failed", str(exc))

    def _update_status_bar(self):
        filename = os.path.basename(self.step_file_path) if self.step_file_path else "No STEP"
        n_points = sum(len(t.points) for t in self.trajectories)
        total_len = sum(t.total_length for t in self.trajectories)
        self.statusBar().showMessage(
            f"Loaded: {filename} | Welds: {len(self.trajectories)} | "
            f"Total points: {n_points} | Total length: {total_len:.2f} mm | "
            f"Step: {self.step_spin.value():.3f} mm"
        )


def launch_with_welds(welds, step_file_path=None, parent=None, step_mm=1.0):
    """
    In-process entry point for main_updated_5.K.py.
    welds must be the live 5.K MainWindow.welds structure, including wire/face objects.
    """
    if not QT_AVAILABLE:
        raise RuntimeError(f"PyQt5 is required for GUI mode: {QT_IMPORT_ERROR}")
    app = QApplication.instance()
    if app is None:
        app = QApplication([])

    window = TrajectoryMappingWindow(parent=parent)
    window.load_welds(welds, step_file_path=step_file_path, step_mm=step_mm)
    window.show()
    return window


def _run_cli(args):
    payload = _load_json_payload(args.input)
    if "trajectories" in payload:
        trajectories = load_trajectory_json(args.input)
        save_trajectory_json(trajectories, args.output)
        print(f"Loaded {len(trajectories)} trajectory object(s); wrote {args.output}")
        return 0

    if "welds" in payload:
        welds, step_path, reconstruction_notes = reconstruct_welds_from_metadata_json(
            payload, json_path=args.input
        )
        trajectories = compute_trajectories(welds, step_mm=args.step)
        if reconstruction_notes:
            for traj in trajectories:
                traj.notes[:0] = reconstruction_notes
        save_trajectory_json(trajectories, args.output)
        print(
            f"Reconstructed {len(welds)} weld(s) from STEP endpoint matches: {step_path}\n"
            f"Wrote {args.output}"
        )
        return 0

    raise RuntimeError("Unsupported input JSON: expected 'trajectories' or 'welds'.")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Trajectory Mapping 1.A")
    parser.add_argument("--input", help="Input JSON path")
    parser.add_argument("--step", type=float, default=1.0, help="Discretization step in mm")
    parser.add_argument("--output", help="Output trajectory JSON path")
    parser.add_argument("--gui", action="store_true", help="Open standalone GUI")
    args = parser.parse_args(argv)

    if args.gui or not args.input:
        if not QT_AVAILABLE:
            raise RuntimeError(f"PyQt5 is required for GUI mode: {QT_IMPORT_ERROR}")
        app = QApplication.instance() or QApplication(sys.argv)
        window = TrajectoryMappingWindow()
        window.show()
        return app.exec_()

    if not args.output:
        raise RuntimeError("--output is required in CLI JSON mode")
    return _run_cli(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
