import sys
import json
import time

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QAction,
    QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QFrame,
    QMessageBox, QRadioButton, QGroupBox, QListWidget, QListWidgetItem,
    QShortcut, QCheckBox, QDialog, QInputDialog, QScrollArea, QSizePolicy
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence, QColor, QBrush, QFont

from OCC.Display.backend import load_backend
load_backend("pyqt5")
from OCC.Display.qtDisplay import qtViewer3d

from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.AIS import AIS_Shape
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX, TopAbs_REVERSED
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopTools import TopTools_HSequenceOfShape
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeVertex,
    BRepBuilderAPI_MakeWire,
)
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepTools import BRepTools_WireExplorer
from OCC.Core.GeomAbs import GeomAbs_Plane
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.Core.BRep import BRep_Tool
from OCC.Core.gp import gp_Pnt, gp_Vec


# ── Parametreler ────────────────────────────────────────────────────────────
PROXIMITY_THRESHOLD_MM   = 5.0
EDGE_MIN_LENGTH          = 0.01
WIRE_TOLERANCE           = 0.5
COINCIDENT_LEN_TOL       = 0.01
COINCIDENT_ENDPOINT_TOL  = 0.1
COINCIDENT_DIST_TOL      = 0.1
CONTINUITY_GAP_TOL       = 0.5
DUPLICATE_TOL            = 0.5

CLEARANCE_THRESHOLD_MM   = 5.0
BRIDGING_MIN_DIST_MM     = 0.05
BRIDGING_MAX_DIST_MM     = 5.0
BRIDGING_LEN_REL_TOL     = 0.15
POINT_CONTACT_TOL_MM     = 1e-3
POINT_CONTACT_CLUSTER_TOL = 0.2
PLANAR_FACING_DOT_MIN    = 0.35

SEGMENT_PALETTE = [
    (0.85, 0.10, 0.10),
    (0.10, 0.10, 0.10),
    (0.10, 0.45, 0.85),
    (0.55, 0.20, 0.75),
    (0.10, 0.55, 0.20),
    (0.85, 0.45, 0.10),
    (0.55, 0.10, 0.45),
    (0.20, 0.55, 0.55),
]

SEL_MODE_SHAPE = 0
SEL_MODE_EDGE  = 2
SEL_MODE_FACE  = 4


# ── Helpers ─────────────────────────────────────────────────────────────────

def _edge_endpoints(edge):
    try:
        result = BRep_Tool.Curve(edge)
        if result is None or len(result) < 3:
            return None
        curve, u_min, u_max = result[0], result[-2], result[-1]
        if curve is None:
            return None
        return (
            curve.Value(u_min),
            curve.Value((u_min + u_max) / 2.0),
            curve.Value(u_max),
        )
    except Exception:
        return None


def _edge_length(edge):
    props = GProp_GProps()
    brepgprop.LinearProperties(edge, props)
    return props.Mass()






def _face_info(face):
    props = GProp_GProps()
    brepgprop.SurfaceProperties(face, props)
    area = props.Mass()
    c = props.CentreOfMass()
    return area, (c.X(), c.Y(), c.Z())


def _wire_endpoints(wire):
    is_closed = BRep_Tool.IsClosed(wire)
    try:
        we = BRepTools_WireExplorer(wire)
        edges_in_order = []
        while we.More():
            edges_in_order.append(we.Current())
            we.Next()
        if not edges_in_order:
            return None, None, is_closed

        first_pts = _edge_endpoints(edges_in_order[0])
        last_pts  = _edge_endpoints(edges_in_order[-1])
        if first_pts is None or last_pts is None:
            return None, None, is_closed

        s = first_pts[0]
        e = last_pts[2]
        return (
            (s.X(), s.Y(), s.Z()),
            (e.X(), e.Y(), e.Z()),
            is_closed,
        )
    except Exception:
        return None, None, is_closed


def _pnt_dist(p1, p2):
    if p1 is None or p2 is None:
        return float("inf")
    dx, dy, dz = p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def _pnt_to_list(p):
    return None if p is None else [p[0], p[1], p[2]]


def _vec_between(p1, p2):
    if p1 is None or p2 is None:
        return None
    return (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])


def _vec_dot(v1, v2):
    if v1 is None or v2 is None:
        return 0.0
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


def _vec_len(v):
    if v is None:
        return 0.0
    return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5


def _vec_normalized(v):
    length = _vec_len(v)
    if length <= 1e-9:
        return None
    return (v[0] / length, v[1] / length, v[2] / length)


def _vec_reversed(v):
    if v is None:
        return None
    return (-v[0], -v[1], -v[2])


# ── Bridging Paths Dialog (5.H: "segment" terminology, default Checked) ─────



class GapEdgePairsDialog(QDialog):
    """5.I: gap durumunda karşılıklı yakın edge çiftlerini seçtirir."""

    def __init__(self, candidates, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Gap Welding Paths")
        self.candidates = candidates
        self.resize(640, 420)

        layout = QVBoxLayout()
        info = QLabel(
            f"Bu iki face arasında <b>{len(candidates)}</b> adet birbirine yakın "
            f"edge çifti tespit edildi.<br>"
            f"Seçilen çiftlerden body numarasına göre ayrı path'ler oluşturulacak."
        )
        info.setWordWrap(True)
        info.setStyleSheet("padding: 8px;")
        layout.addWidget(info)

        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet("QListWidget::item { padding: 8px; }")
        for i, c in enumerate(candidates, 1):
            suffix = "  [recommended]" if c.get("recommended", True) else ""
            text = (
                f"Pair {i} — Body {c['body_a']}: {c['length_a']:.2f} mm  |  "
                f"Body {c['body_b']}: {c['length_b']:.2f} mm  |  "
                f"gap {c['dist']:.2f} mm{suffix}"
            )
            item = QListWidgetItem(text)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if c.get("recommended", True) else Qt.Unchecked)
            self.list_widget.addItem(item)
        layout.addWidget(self.list_widget)

        warn = QLabel(
            "⚠ Gap path'leri otomatik tespit edilir.\n"
            "Path harfi body numarasına göre atanır: Body 1 → Path A, Body 2 → Path B. "
            "Tam karşılıklı yüz değilse sadece ana seam pair'i varsayılan seçilir."
        )
        warn.setStyleSheet(
            "color: #888; font-style: italic; font-size: 11px; padding: 4px;"
        )
        warn.setWordWrap(True)
        layout.addWidget(warn)

        btn_layout = QHBoxLayout()
        self.btn_add = QPushButton("Add Selected to Body Paths")
        self.btn_add.clicked.connect(self.accept)
        self.btn_add.setDefault(True)
        btn_layout.addWidget(self.btn_add)

        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.clicked.connect(self.reject)
        btn_layout.addWidget(self.btn_cancel)
        layout.addLayout(btn_layout)

        self.setLayout(layout)

    def selected_indices(self):
        result = []
        for i in range(self.list_widget.count()):
            item = self.list_widget.item(i)
            if item.checkState() == Qt.Checked:
                result.append(i)
        return result


# ── Viewer ──────────────────────────────────────────────────────────────────

class WeldingViewer(qtViewer3d):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.on_shape_picked = None

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._display.Select(event.pos().x(), event.pos().y())
            ctx = self._display.GetContext()
            if ctx.NbSelected() > 0:
                ctx.InitSelected()
                while ctx.MoreSelected():
                    if self.on_shape_picked:
                        self.on_shape_picked(ctx.SelectedShape())
                    ctx.NextSelected()
        super().mousePressEvent(event)


# ── Main Window ─────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    """5.K: simplified weld/segment model with facing-face gap filter."""

    COLOR_FACE_A   = Quantity_Color(1.00, 0.85, 0.00, Quantity_TOC_RGB)
    COLOR_FACE_B   = Quantity_Color(0.10, 0.70, 1.00, Quantity_TOC_RGB)
    COLOR_CYAN     = Quantity_Color(0.00, 0.85, 1.00, Quantity_TOC_RGB)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Laser Welding – Phase 3 (Paths & Segments)")
        self.resize(1380, 820)

        self.viewer = WeldingViewer(self)
        self.viewer.on_shape_picked = self._on_shape_picked
        self.side_panel = self._create_side_panel()

        container = QWidget()
        layout = QHBoxLayout()
        layout.addWidget(self.viewer, stretch=4)
        layout.addWidget(self.side_panel, stretch=2)
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.solids = []
        self.ais_shapes = []
        self.body_colors = []
        self.current_step_file = ""

        self.face_a = None
        self.face_b = None

        self.manual_edges = []
        self.manual_ais_list = []

        self.welds = []
        self.active_weld_id = None
        self.collected_segments = []
        self.loaded_metadata = None

        self._create_menu()
        self._install_shortcuts()
        self._update_status_bar()

    # ─── Menu ──────────────────────────────────────────────────────────

    def _create_menu(self):
        mb = self.menuBar()
        file_menu = mb.addMenu("File")

        open_action = QAction("Open STEP...", self)
        open_action.setShortcut(QKeySequence("Ctrl+O"))
        open_action.triggered.connect(self.open_step_file)
        file_menu.addAction(open_action)

        file_menu.addSeparator()

        save_action = QAction("Save Path As JSON...", self)
        save_action.setShortcut(QKeySequence("Ctrl+S"))
        save_action.triggered.connect(self.save_path_json)
        file_menu.addAction(save_action)

        load_action = QAction("Load Path From JSON...", self)
        load_action.setShortcut(QKeySequence("Ctrl+L"))
        load_action.triggered.connect(self.load_path_json)
        file_menu.addAction(load_action)

        export_action = QAction("Export Path for Trajectory Planning...", self)
        export_action.setShortcut(QKeySequence("Ctrl+E"))
        export_action.triggered.connect(self.export_path_for_trajectory)
        file_menu.addAction(export_action)

    # ─── Shortcuts ─────────────────────────────────────────────────────

    def _install_shortcuts(self):
        QShortcut(QKeySequence(Qt.Key_Return), self, activated=self._sc_add_segment)
        QShortcut(QKeySequence(Qt.Key_Enter),  self, activated=self._sc_add_segment)
        QShortcut(QKeySequence(Qt.Key_Space),  self, activated=self._sc_add_segment)
        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.reset_face_selection)
        QShortcut(QKeySequence(Qt.Key_Delete), self, activated=self._sc_delete)
        QShortcut(QKeySequence("Ctrl+Z"),      self, activated=self.undo_last_segment)

    def _sc_add_segment(self):
        if (self.radio_auto.isChecked()
                and self.face_a is not None and self.face_b is not None):
            self.add_segment()

    def _sc_delete(self):
        if self.segment_list.selectedItems():
            self.delete_selected_segment()

    # ─── Weld model ─────────────────────────────────────────────────────

    def _active_weld(self):
        if self.active_weld_id is None:
            return None
        for weld in self.welds:
            if weld["id"] == self.active_weld_id:
                return weld
        return None

    def _active_segments(self):
        """Aktif weld'in segment listesi. Active weld yoksa boş liste döner."""
        weld = self._active_weld()
        if weld is None:
            return []
        return weld["segments"]

    def _sync_collected_segments(self):
        self.collected_segments = self._active_segments()

    def _next_weld_id(self):
        if not self.welds:
            return 1
        return max(w["id"] for w in self.welds) + 1

    def _create_new_weld(self, name=None, context="manual"):
        next_id = self._next_weld_id()
        if name is None:
            name = f"Path {next_id}"
        weld = {
            "id": next_id,
            "name": name,
            "segments": [],
            "created_at": time.time(),
            "context": context,
        }
        self.welds.append(weld)
        self.active_weld_id = next_id
        self._sync_collected_segments()
        self._reassign_segment_colors()
        self._refresh_weld_list()
        self._rebuild_segment_list()
        self._update_segments_label()
        self._update_status_bar()
        return weld

    def new_weld_from_button(self):
        default = f"Path {self._next_weld_id()}"
        name, ok = QInputDialog.getText(self, "New Path", "Path name:", text=default)
        if not ok:
            return
        name = name.strip() or default
        self._create_new_weld(name=name)

    def _refresh_weld_list(self):
        self.weld_list.blockSignals(True)
        self.weld_list.clear()
        for weld in self.welds:
            segments = weld["segments"]
            total = sum(s.get("length", 0.0) for s in segments)
            text = (
                f"#{weld['id']} — {weld['name']}  "
                f"({len(segments)} segments, {total:.1f} mm)"
            )
            item = QListWidgetItem(text)
            item.setData(Qt.UserRole, weld["id"])
            if weld["id"] == self.active_weld_id:
                font = item.font()
                font.setBold(True)
                item.setFont(font)
            self.weld_list.addItem(item)
            if weld["id"] == self.active_weld_id:
                self.weld_list.setCurrentItem(item)
        self.weld_list.blockSignals(False)
        has_weld = self.active_weld_id is not None
        self.btn_rename_weld.setEnabled(has_weld)
        self.btn_delete_weld.setEnabled(has_weld)

    def _on_weld_selected(self):
        items = self.weld_list.selectedItems()
        if not items:
            return
        weld_id = items[0].data(Qt.UserRole)
        if weld_id == self.active_weld_id:
            return
        self.active_weld_id = weld_id
        self._sync_collected_segments()
        self._refresh_weld_list()
        self._reassign_segment_colors()
        self._rebuild_segment_list()
        self._refresh_weld_list()
        self._update_segments_label()
        self._update_status()
        self._update_status_bar()

    def rename_selected_weld(self):
        weld = self._active_weld()
        if weld is None:
            return
        name, ok = QInputDialog.getText(
            self, "Rename Path", "Path name:", text=weld["name"]
        )
        if not ok:
            return
        name = name.strip()
        if not name:
            return
        weld["name"] = name
        self._refresh_weld_list()
        self._update_status_bar()

    def delete_selected_weld(self):
        weld = self._active_weld()
        if weld is None:
            return
        ctx = self.viewer._display.GetContext()
        for seg in weld["segments"]:
            for ais in seg.get("ais_list", []):
                ctx.Remove(ais, False)
        self.welds = [w for w in self.welds if w["id"] != weld["id"]]
        self.active_weld_id = self.welds[0]["id"] if self.welds else None
        self._sync_collected_segments()
        self.viewer._display.Repaint()
        self._refresh_weld_list()
        self._reassign_segment_colors()
        self._rebuild_segment_list()
        self._update_segments_label()
        self._update_status()
        self._update_status_bar()

    # ─── Side panel ────────────────────────────────────────────────────

    def _create_side_panel(self):
        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        panel.setMinimumWidth(360)
        v = QVBoxLayout()
        v.setSpacing(8)
        v.setContentsMargins(12, 12, 12, 12)

        mode_box = QGroupBox("Detection Mode")
        ml = QVBoxLayout()
        self.radio_auto   = QRadioButton("Auto (pick 2 faces)")
        self.radio_manual = QRadioButton("Manual (pick edges)")
        self.radio_auto.setChecked(True)
        self.radio_auto.toggled.connect(self._on_mode_toggled)
        ml.addWidget(self.radio_auto)
        ml.addWidget(self.radio_manual)
        self.chk_show_details = QCheckBox("Show details after each segment")
        self.chk_show_details.setChecked(False)
        ml.addWidget(self.chk_show_details)
        mode_box.setLayout(ml)
        v.addWidget(mode_box)

        self.lbl_status = QLabel("Open a STEP file to begin.")
        self.lbl_status.setWordWrap(True)
        self.lbl_status.setStyleSheet(
            "padding: 8px; background: #f0f0f0; color: #333;"
        )
        v.addWidget(self.lbl_status)

        self.lbl_face_a = QLabel("Face X:  —")
        self.lbl_face_a.setWordWrap(True)
        self.lbl_face_a.setStyleSheet(
            "padding: 4px; color: #b08000; font-weight: bold;"
        )
        v.addWidget(self.lbl_face_a)

        self.lbl_face_b = QLabel("Face Y:  —")
        self.lbl_face_b.setWordWrap(True)
        self.lbl_face_b.setStyleSheet(
            "padding: 4px; color: #006090; font-weight: bold;"
        )
        v.addWidget(self.lbl_face_b)

        self.lbl_proximity = QLabel("")
        self.lbl_proximity.setWordWrap(True)
        self.lbl_proximity.setStyleSheet(
            "padding: 4px; color: #004400; font-weight: bold;"
        )
        v.addWidget(self.lbl_proximity)

        self.btn_reset_faces = QPushButton("Reset Faces")
        self.btn_reset_faces.setToolTip("Clear face X/Y (Esc)")
        self.btn_reset_faces.clicked.connect(self.reset_face_selection)
        v.addWidget(self.btn_reset_faces)

        self.btn_add_segment = QPushButton("Add Segment")
        self.btn_add_segment.setToolTip("Add segment from current face pair (Enter)")
        self.btn_add_segment.clicked.connect(self.add_segment)
        self.btn_add_segment.setEnabled(False)
        v.addWidget(self.btn_add_segment)

        seg_header_layout = QHBoxLayout()
        self.lbl_segments = QLabel("Segments: 0  |  Total: 0.0 mm")
        self.lbl_segments.setStyleSheet(
            "padding: 4px; background: #fff8e1; color: #555; font-weight: bold;"
        )
        seg_header_layout.addWidget(self.lbl_segments, 1)
        self.lbl_legend = QLabel("Colors: distinct per segment")
        self.lbl_legend.setStyleSheet("padding: 4px; color: #888; font-style: italic;")
        seg_header_layout.addWidget(self.lbl_legend)
        v.addLayout(seg_header_layout)

        self.segment_list = QListWidget()
        self.segment_list.setMinimumHeight(160)
        self.segment_list.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.segment_list.itemSelectionChanged.connect(self._on_segment_selected)
        v.addWidget(self.segment_list)

        list_ctrl_layout = QHBoxLayout()
        self.btn_delete_segment = QPushButton("Delete")
        self.btn_delete_segment.setToolTip("Delete selected segment (Del)")
        self.btn_delete_segment.clicked.connect(self.delete_selected_segment)
        self.btn_delete_segment.setEnabled(False)
        list_ctrl_layout.addWidget(self.btn_delete_segment)

        self.btn_move_up = QPushButton("↑ Up")
        self.btn_move_up.clicked.connect(self.move_segment_up)
        self.btn_move_up.setEnabled(False)
        list_ctrl_layout.addWidget(self.btn_move_up)

        self.btn_move_down = QPushButton("↓ Down")
        self.btn_move_down.clicked.connect(self.move_segment_down)
        self.btn_move_down.setEnabled(False)
        list_ctrl_layout.addWidget(self.btn_move_down)
        v.addLayout(list_ctrl_layout)

        self.btn_undo = QPushButton("Undo Last Segment")
        self.btn_undo.setToolTip("Remove last added segment (Ctrl+Z)")
        self.btn_undo.clicked.connect(self.undo_last_segment)
        self.btn_undo.setEnabled(False)
        v.addWidget(self.btn_undo)

        self.btn_finish = QPushButton("Finalize All Segments")
        self.btn_finish.clicked.connect(self.finish_path)
        self.btn_finish.setEnabled(False)
        v.addWidget(self.btn_finish)

        self.btn_clear_segments = QPushButton("Clear All Segments")
        self.btn_clear_segments.clicked.connect(self.clear_all_segments)
        self.btn_clear_segments.setEnabled(False)
        v.addWidget(self.btn_clear_segments)

        weld_box = QGroupBox("Paths")
        wl = QVBoxLayout()
        wl.setSpacing(6)
        self.weld_list = QListWidget()
        self.weld_list.setMinimumHeight(90)
        self.weld_list.setMaximumHeight(140)
        self.weld_list.itemSelectionChanged.connect(self._on_weld_selected)
        wl.addWidget(self.weld_list)

        weld_ctrl_layout = QHBoxLayout()
        self.btn_new_weld = QPushButton("+ New Path")
        self.btn_new_weld.clicked.connect(self.new_weld_from_button)
        weld_ctrl_layout.addWidget(self.btn_new_weld)

        self.btn_rename_weld = QPushButton("Rename")
        self.btn_rename_weld.clicked.connect(self.rename_selected_weld)
        self.btn_rename_weld.setEnabled(False)
        weld_ctrl_layout.addWidget(self.btn_rename_weld)

        self.btn_delete_weld = QPushButton("Delete")
        self.btn_delete_weld.clicked.connect(self.delete_selected_weld)
        self.btn_delete_weld.setEnabled(False)
        weld_ctrl_layout.addWidget(self.btn_delete_weld)
        wl.addLayout(weld_ctrl_layout)
        weld_box.setLayout(wl)
        v.addWidget(weld_box)
        self.weld_box = weld_box

        self.btn_clear_edges = QPushButton("Clear Edges")
        self.btn_clear_edges.clicked.connect(self._clear_manual_edges)
        self.btn_clear_edges.setVisible(False)
        v.addWidget(self.btn_clear_edges)

        self.btn_apply_manual = QPushButton("Apply Manual Path")
        self.btn_apply_manual.clicked.connect(self.apply_manual)
        self.btn_apply_manual.setEnabled(False)
        self.btn_apply_manual.setVisible(False)
        v.addWidget(self.btn_apply_manual)

        self.lbl_path = QLabel("")
        self.lbl_path.setWordWrap(True)
        self.lbl_path.setStyleSheet("padding: 4px; color: #222;")
        v.addWidget(self.lbl_path)

        v.addStretch()
        panel.setLayout(v)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(380)
        scroll.setFrameShape(QFrame.StyledPanel)
        scroll.setWidget(panel)
        return scroll

    # ─── Mode switching ────────────────────────────────────────────────

    def _activate_selection_mode(self, mode):
        ctx = self.viewer._display.GetContext()
        for ais in self.ais_shapes:
            for m in (SEL_MODE_SHAPE, SEL_MODE_EDGE, SEL_MODE_FACE):
                if m == mode:
                    ctx.Activate(ais, m, False)
                else:
                    ctx.Deactivate(ais, m)

    def _on_mode_toggled(self, auto_checked):
        if auto_checked:
            self._activate_selection_mode(SEL_MODE_FACE)
            for w in (self.btn_add_segment, self.btn_finish, self.btn_undo,
                      self.btn_clear_segments, self.btn_reset_faces,
                      self.btn_delete_segment, self.btn_move_up,
                      self.btn_move_down, self.lbl_face_a, self.lbl_face_b,
                      self.lbl_segments, self.lbl_legend, self.segment_list,
                      self.weld_box):
                w.setVisible(True)
            self.btn_clear_edges.setVisible(False)
            self.btn_apply_manual.setVisible(False)
        else:
            self._activate_selection_mode(SEL_MODE_EDGE)
            for w in (self.btn_add_segment, self.btn_finish, self.btn_undo,
                      self.btn_clear_segments, self.btn_delete_segment,
                      self.btn_move_up, self.btn_move_down,
                      self.lbl_face_a, self.lbl_face_b, self.lbl_segments,
                      self.lbl_legend, self.segment_list, self.weld_box):
                w.setVisible(False)
            self.btn_clear_edges.setVisible(True)
            self.btn_apply_manual.setVisible(True)
        self._update_status()

    # ─── File loading ──────────────────────────────────────────────────

    def init_viewer(self):
        self.viewer.InitDriver()

    def open_step_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open STEP file", "",
            "STEP files (*.step *.stp *.STEP *.STP)"
        )
        if not path:
            return
        reader = STEPControl_Reader()
        if reader.ReadFile(path) != IFSelect_RetDone:
            QMessageBox.critical(self, "Error", f"Cannot read:\n{path}")
            return
        reader.TransferRoots()
        root = reader.OneShape()
        self.solids = list(TopologyExplorer(root).solids())
        self.current_step_file = path
        print(f"[FILE] Loaded {len(self.solids)} solid(s) from {path}")

        self.viewer._display.EraseAll()
        self.ais_shapes.clear()
        self.body_colors.clear()
        self.face_a = None
        self.face_b = None
        self.manual_edges.clear()
        self.manual_ais_list.clear()
        self.welds.clear()
        self.active_weld_id = None
        self._sync_collected_segments()
        self.lbl_proximity.setText("")
        self.lbl_path.setText("")

        palette = [
            Quantity_Color(0.55, 0.55, 0.55, Quantity_TOC_RGB),
            Quantity_Color(0.40, 0.50, 0.70, Quantity_TOC_RGB),
            Quantity_Color(0.70, 0.50, 0.40, Quantity_TOC_RGB),
            Quantity_Color(0.40, 0.65, 0.45, Quantity_TOC_RGB),
            Quantity_Color(0.65, 0.45, 0.65, Quantity_TOC_RGB),
            Quantity_Color(0.50, 0.65, 0.65, Quantity_TOC_RGB),
        ]
        for idx, solid in enumerate(self.solids):
            color = palette[idx % len(palette)]
            result = self.viewer._display.DisplayShape(solid, color=color, update=False)
            ais = result[0] if isinstance(result, list) else result
            self.ais_shapes.append(ais)
            self.body_colors.append(color)

        if self.radio_auto.isChecked():
            self._activate_selection_mode(SEL_MODE_FACE)
        else:
            self._activate_selection_mode(SEL_MODE_EDGE)

        self.viewer._display.FitAll()
        self._refresh_weld_list()
        self._rebuild_segment_list()
        self._update_status()
        self._update_segments_label()
        self._update_status_bar()

    # ─── Pick dispatch ─────────────────────────────────────────────────

    def _on_shape_picked(self, shape):
        if self.radio_auto.isChecked():
            self._pick_face(shape)
        else:
            self._pick_edge(shape)

    # ─── Face picking ──────────────────────────────────────────────────

    def _pick_face(self, face):
        if face.ShapeType() != TopAbs_FACE:
            return
        body_idx = self._find_body_of_face(face)
        if body_idx is None:
            return

        if self.face_a is not None and self.face_a[0].IsSame(face):
            self._remove_face_highlight(self.face_a[2])
            self.face_a = None
            self._update_status()
            return
        if self.face_b is not None and self.face_b[0].IsSame(face):
            self._remove_face_highlight(self.face_b[2])
            self.face_b = None
            self._update_status()
            return

        if self.face_a is None:
            self._set_face_a(face, body_idx)
        elif self.face_b is None:
            if body_idx == self.face_a[1]:
                QMessageBox.information(
                    self, "Same Body",
                    f"Bu face Body {body_idx}'den; Face X de aynı body'den.\n"
                    f"Lütfen FARKLI bir body'den face seçin."
                )
                return
            self._set_face_b(face, body_idx)
        else:
            self._remove_face_highlight(self.face_a[2])
            of, ob, oa, oar, oc = self.face_b
            self.viewer._display.GetContext().SetColor(oa, self.COLOR_FACE_A, True)
            self.face_a = (of, ob, oa, oar, oc)
            self.face_b = None
            if body_idx == self.face_a[1]:
                QMessageBox.information(
                    self, "Same Body",
                    f"Bu face Body {body_idx}'den; Face X de aynı body'den.\n"
                    f"Lütfen FARKLI bir body'den face seçin."
                )
                self._update_status()
                return
            self._set_face_b(face, body_idx)

        self._update_status()

    def _set_face_a(self, face, body_idx):
        area, center = _face_info(face)
        ais = self._highlight_face(face, self.COLOR_FACE_A)
        self.face_a = (face, body_idx, ais, area, center)
        print(f"[PICK] Face X → Body {body_idx}, area={area:.2f} mm², center={center}")

    def _set_face_b(self, face, body_idx):
        area, center = _face_info(face)
        ais = self._highlight_face(face, self.COLOR_FACE_B)
        self.face_b = (face, body_idx, ais, area, center)
        print(f"[PICK] Face Y → Body {body_idx}, area={area:.2f} mm², center={center}")

    def _highlight_face(self, face, color):
        ais = AIS_Shape(face)
        ais.SetColor(color)
        ctx = self.viewer._display.GetContext()
        ctx.Display(ais, False)
        ctx.Deactivate(ais)
        self.viewer._display.Repaint()
        return ais

    def _remove_face_highlight(self, ais):
        ctx = self.viewer._display.GetContext()
        ctx.Remove(ais, False)
        self.viewer._display.Repaint()

    def _find_body_of_face(self, face):
        for i, solid in enumerate(self.solids):
            exp = TopExp_Explorer(solid, TopAbs_FACE)
            while exp.More():
                if exp.Current().IsSame(face):
                    return i + 1
                exp.Next()
        return None

    # ─── Edge picking (Manual) ─────────────────────────────────────────

    def _pick_edge(self, edge):
        if edge.ShapeType() != TopAbs_EDGE:
            return
        length = _edge_length(edge)
        print(f"[PICK] Manual edge: {length:.2f} mm")
        self.manual_edges.append(edge)
        ais = AIS_Shape(edge)
        ais.SetColor(self.COLOR_CYAN)
        ais.SetWidth(4.0)
        self.viewer._display.GetContext().Display(ais, True)
        self.manual_ais_list.append(ais)
        self.btn_apply_manual.setEnabled(True)
        self.lbl_status.setText(f"Edges collected: {len(self.manual_edges)}")

    # ─── Reset / clear ─────────────────────────────────────────────────

    def reset_face_selection(self):
        ctx = self.viewer._display.GetContext()
        if self.face_a is not None:
            ctx.Remove(self.face_a[2], False)
            self.face_a = None
        if self.face_b is not None:
            ctx.Remove(self.face_b[2], False)
            self.face_b = None
        self.viewer._display.Repaint()
        self.lbl_proximity.setText("")
        self._update_status()

    def clear_all_segments(self):
        ctx = self.viewer._display.GetContext()
        segments = self._active_segments()
        for seg in segments:
            for ais in seg.get("ais_list", []):
                ctx.Remove(ais, False)
        segments.clear()
        self.viewer._display.Repaint()
        self.lbl_path.setText("")
        self._rebuild_segment_list()
        self._refresh_weld_list()
        self._update_segments_label()
        self._update_status()
        self._update_status_bar()
        print("[STATE] All segments cleared.")

    def _clear_manual_edges(self):
        ctx = self.viewer._display.GetContext()
        for ais in self.manual_ais_list:
            ctx.Remove(ais, False)
        self.viewer._display.Repaint()
        self.manual_edges.clear()
        self.manual_ais_list.clear()
        self.btn_apply_manual.setEnabled(False)
        if self.radio_manual.isChecked():
            self.lbl_status.setText("Manual mode:\nClick edges on the model.")

    # ─── Validation ────────────────────────────────────────────────────

    def _validate_face_proximity(self):
        face_a = self.face_a[0]
        face_b = self.face_b[0]
        d = BRepExtrema_DistShapeShape(face_a, face_b)
        d.Perform()
        if not d.IsDone():
            return None, "Distance computation failed."
        gap = d.Value()
        if gap > PROXIMITY_THRESHOLD_MM:
            return gap, (
                f"⚠ Faces {gap:.2f} mm apart "
                f"(no shared edge expected)"
            )
        return gap, ""

    def _face_surface_type(self, face):
        try:
            return BRepAdaptor_Surface(face, True).GetType()
        except Exception as e:
            print(f"[ALGO] Surface type failed: {e}")
            return None

    def _face_normal_at_midpoint(self, face):
        try:
            surf = BRepAdaptor_Surface(face, True)
            u = (surf.FirstUParameter() + surf.LastUParameter()) / 2.0
            v = (surf.FirstVParameter() + surf.LastVParameter()) / 2.0
            p = gp_Pnt()
            du = gp_Vec()
            dv = gp_Vec()
            surf.D1(u, v, p, du, dv)
            normal = du.Crossed(dv)
            if normal.Magnitude() <= 1e-9:
                return None
            normal.Normalize()
            if face.Orientation() == TopAbs_REVERSED:
                normal.Reverse()
            return (normal.X(), normal.Y(), normal.Z())
        except Exception as e:
            print(f"[ALGO] Face normal failed: {e}")
            return None

    def _faces_suitable_for_gap_paths(self, face_a, face_b):
        type_a = self._face_surface_type(face_a)
        type_b = self._face_surface_type(face_b)

        if type_a != GeomAbs_Plane or type_b != GeomAbs_Plane:
            return False

        normal_a = self._face_normal_at_midpoint(face_a)
        normal_b = self._face_normal_at_midpoint(face_b)
        _, center_a = _face_info(face_a)
        _, center_b = _face_info(face_b)
        direction = _vec_normalized(_vec_between(center_a, center_b))
        if normal_a is None or normal_b is None or direction is None:
            return False
        return (
            _vec_dot(normal_a, direction) >= PLANAR_FACING_DOT_MIN and
            _vec_dot(normal_b, _vec_reversed(direction)) >= PLANAR_FACING_DOT_MIN and
            _vec_dot(normal_a, normal_b) <= -PLANAR_FACING_DOT_MIN
        )

    # ─── ALGO 1: Topological shared edges ─────────────────────────────

    def _find_shared_edges(self, face_a, face_b):
        edges_a = []
        exp = TopExp_Explorer(face_a, TopAbs_EDGE)
        while exp.More():
            edges_a.append(exp.Current())
            exp.Next()
        shared = []
        exp = TopExp_Explorer(face_b, TopAbs_EDGE)
        while exp.More():
            edge_b = exp.Current()
            for edge_a in edges_a:
                if edge_a.IsSame(edge_b):
                    length = _edge_length(edge_b)
                    if length >= EDGE_MIN_LENGTH:
                        shared.append((edge_b, length))
                    break
            exp.Next()
        return shared

    # ─── ALGO 2: Geometric coincident edges ───────────────────────────

    def _find_coincident_edges(self, face_a, face_b):
        edges_a = []
        exp = TopExp_Explorer(face_a, TopAbs_EDGE)
        while exp.More():
            edges_a.append(exp.Current())
            exp.Next()
        edges_b = []
        exp = TopExp_Explorer(face_b, TopAbs_EDGE)
        while exp.More():
            edges_b.append(exp.Current())
            exp.Next()

        coincident = []
        matched_b = set()
        for edge_a in edges_a:
            length_a = _edge_length(edge_a)
            if length_a < EDGE_MIN_LENGTH:
                continue
            ends_a = _edge_endpoints(edge_a)
            if ends_a is None:
                continue
            pa_start, _, pa_end = ends_a

            for j, edge_b in enumerate(edges_b):
                if j in matched_b:
                    continue
                length_b = _edge_length(edge_b)
                if abs(length_a - length_b) > COINCIDENT_LEN_TOL:
                    continue
                ends_b = _edge_endpoints(edge_b)
                if ends_b is None:
                    continue
                pb_start, _, pb_end = ends_b
                forward = (
                    pa_start.Distance(pb_start) < COINCIDENT_ENDPOINT_TOL and
                    pa_end.Distance(pb_end)     < COINCIDENT_ENDPOINT_TOL
                )
                reverse = (
                    pa_start.Distance(pb_end)   < COINCIDENT_ENDPOINT_TOL and
                    pa_end.Distance(pb_start)   < COINCIDENT_ENDPOINT_TOL
                )
                if not (forward or reverse):
                    continue
                d = BRepExtrema_DistShapeShape(edge_a, edge_b)
                d.Perform()
                if not d.IsDone() or d.Value() > COINCIDENT_DIST_TOL:
                    continue
                coincident.append((edge_a, length_a))
                matched_b.add(j)
                break
        return coincident

    # ─── ALGO 3: Section fallback ──────────────────────────────────────

    def _face_vertices(self, face):
        vertices = []
        exp = TopExp_Explorer(face, TopAbs_VERTEX)
        while exp.More():
            try:
                p = BRep_Tool.Pnt(exp.Current())
                pt = (p.X(), p.Y(), p.Z())
                if not any(_pnt_dist(pt, existing) < POINT_CONTACT_CLUSTER_TOL
                           for existing in vertices):
                    vertices.append(pt)
            except Exception:
                pass
            exp.Next()
        return vertices

    def _cluster_points(self, points):
        clusters = []
        for pt in points:
            if pt is None:
                continue
            if any(_pnt_dist(pt, existing) < POINT_CONTACT_CLUSTER_TOL
                   for existing in clusters):
                continue
            clusters.append(pt)
        return clusters

    def _find_point_contact(self, face_a, face_b):
        points = []

        try:
            d = BRepExtrema_DistShapeShape(face_a, face_b)
            d.Perform()
            if d.IsDone() and d.Value() <= POINT_CONTACT_TOL_MM:
                for i in range(1, d.NbSolution() + 1):
                    p1 = d.PointOnShape1(i)
                    p2 = d.PointOnShape2(i)
                    if p1.Distance(p2) <= POINT_CONTACT_TOL_MM:
                        points.append((
                            (p1.X() + p2.X()) / 2.0,
                            (p1.Y() + p2.Y()) / 2.0,
                            (p1.Z() + p2.Z()) / 2.0,
                        ))
        except Exception as e:
            print(f"[ALGO] Point-contact extrema failed: {e}")

        verts_a = self._face_vertices(face_a)
        verts_b = self._face_vertices(face_b)
        for pa in verts_a:
            for pb in verts_b:
                if _pnt_dist(pa, pb) <= POINT_CONTACT_TOL_MM:
                    points.append((
                        (pa[0] + pb[0]) / 2.0,
                        (pa[1] + pb[1]) / 2.0,
                        (pa[2] + pb[2]) / 2.0,
                    ))

        clusters = self._cluster_points(points)
        if len(clusters) == 1:
            return clusters[0]
        if clusters:
            print(f"[ALGO] Point-contact candidates are not unique: {len(clusters)}")
        return None

    def _section_fallback(self, face_a, face_b):
        section = BRepAlgoAPI_Section(face_a, face_b)
        section.ComputePCurveOn1(False)
        section.Approximation(False)
        section.Build()
        if not section.IsDone():
            return []
        result = section.Shape()
        if result.IsNull():
            return []
        edges = []
        exp = TopExp_Explorer(result, TopAbs_EDGE)
        while exp.More():
            edge = exp.Current()
            length = _edge_length(edge)
            if length >= EDGE_MIN_LENGTH:
                edges.append((edge, length))
            exp.Next()
        return edges

    # ─── ALGO 4: Path-based bridging ──────────────────────────────────


    def _face_edges(self, face):
        edges = []
        exp = TopExp_Explorer(face, TopAbs_EDGE)
        while exp.More():
            edge = exp.Current()
            length = _edge_length(edge)
            if length >= EDGE_MIN_LENGTH:
                edges.append((edge, length))
            exp.Next()
        return edges

    def _mark_recommended_gap_pairs(self, candidates):
        if not candidates:
            return candidates

        # Tam karşılıklı iki dikdörtgen/simetrik face seçildiyse genelde 4 kenar
        # ve uzunluk grupları dengelidir. Kısmi/yan seçilimde ise kısa yan
        # kenarlar da mesafe filtresine takılabilir; default olarak ana seam'i seç.
        lengths = [max(c["length_a"], c["length_b"]) for c in candidates]
        max_len = max(lengths)
        groups = []
        for length in sorted(lengths, reverse=True):
            for group in groups:
                ref = group[0]
                if abs(length - ref) <= max(0.5, ref * 0.08):
                    group.append(length)
                    break
            else:
                groups.append([length])

        looks_like_full_boundary = (
            len(candidates) >= 4 and
            (
                len(groups) == 1 or
                sum(1 for group in groups if len(group) >= 2) >= 2
            )
        )

        for c in candidates:
            length = max(c["length_a"], c["length_b"])
            c["recommended"] = (
                True if looks_like_full_boundary
                else abs(length - max_len) <= max(0.5, max_len * 0.08)
            )
        return candidates

    def _find_gap_edge_pairs(self, face_a, face_b):
        body_a_idx = self.face_a[1] if self.face_a is not None else 0
        body_b_idx = self.face_b[1] if self.face_b is not None else 0
        edges_a = self._face_edges(face_a)
        edges_b = self._face_edges(face_b)
        if not edges_a or not edges_b:
            return []

        candidates = []
        matched_b = set()
        for edge_a, length_a in edges_a:
            best_for_a = None
            for j, (edge_b, length_b) in enumerate(edges_b):
                if j in matched_b:
                    continue
                avg_len = (length_a + length_b) / 2.0
                if avg_len > 0 and abs(length_a - length_b) / avg_len > BRIDGING_LEN_REL_TOL:
                    continue
                d = BRepExtrema_DistShapeShape(edge_a, edge_b)
                d.Perform()
                if not d.IsDone():
                    continue
                dist = d.Value()
                if dist < BRIDGING_MIN_DIST_MM or dist > BRIDGING_MAX_DIST_MM:
                    continue
                if best_for_a is None or dist < best_for_a[3]:
                    best_for_a = (j, edge_b, length_b, dist)

            if best_for_a is not None:
                j, edge_b, length_b, dist = best_for_a
                candidates.append({
                    "edge_a": edge_a,
                    "edge_b": edge_b,
                    "length_a": length_a,
                    "length_b": length_b,
                    "dist": dist,
                    "body_a": body_a_idx,
                    "body_b": body_b_idx,
                    "side_a": self._path_letter_for_body(body_a_idx),
                    "side_b": self._path_letter_for_body(body_b_idx),
                })
                matched_b.add(j)

        candidates.sort(key=lambda c: (round(c["dist"], 3), -max(c["length_a"], c["length_b"])))
        return self._mark_recommended_gap_pairs(candidates)



    # ─── 5.H: side-aware bridging duplicate check ─────────────────────



    def _segment_from_edge(self, edge, length, method_used, pair_idx):
        wires = self._build_wires([(edge, length)])
        if not wires:
            return None
        primary = wires[0]
        start_pt, end_pt, is_closed = _wire_endpoints(primary["wire"])
        total_length = sum(w["length"] for w in wires)
        return {
            "wires": wires,
            "ais_list": [],
            "method": method_used,
            "length": total_length,
            "type": self._path_type_label(primary),
            "start_point": start_pt,
            "end_point": end_pt,
            "is_closed": is_closed,
            "gap_pair_index": pair_idx,
        }

    def _display_segment_for_weld(self, weld, seg):
        ctx = self.viewer._display.GetContext()
        seg_idx = len(weld["segments"])
        color = self._segment_color(seg_idx)
        ais_list = []
        for w in seg["wires"]:
            ais = AIS_Shape(w["wire"])
            ais.SetColor(color)
            ais.SetWidth(5.0)
            ctx.Display(ais, False)
            ais_list.append(ais)
        seg["ais_list"] = ais_list
        weld["segments"].append(seg)


    def _path_for_gap_side(self, side, body_idx, prefer_active=False):
        active = self._active_weld()
        if prefer_active and active is not None and not active["segments"]:
            active["name"] = f"Path {side} - Body {body_idx} side"
            active["context"] = "bridging"
            return active
        return self._create_new_weld(
            name=f"Path {side} - Body {body_idx} side",
            context="bridging"
        )

    def _apply_gap_edge_pairs(self, candidates, selected_indices):
        if not selected_indices:
            return

        side_a = candidates[0]["side_a"]
        side_b = candidates[0]["side_b"]
        path_a = self._path_for_gap_side(side_a, candidates[0]["body_a"], prefer_active=True)
        path_b = self._path_for_gap_side(side_b, candidates[0]["body_b"])
        added_by_side = {side_a: 0, side_b: 0}

        for idx in selected_indices:
            c = candidates[idx]
            method_a = f"Bridging-{c['side_a']} (gap {c['dist']:.2f} mm)"
            method_b = f"Bridging-{c['side_b']} (gap {c['dist']:.2f} mm)"
            seg_a = self._segment_from_edge(c["edge_a"], c["length_a"], method_a, idx + 1)
            seg_b = self._segment_from_edge(c["edge_b"], c["length_b"], method_b, idx + 1)
            if seg_a is not None:
                self._display_segment_for_weld(path_a, seg_a)
                added_by_side[c["side_a"]] += 1
            if seg_b is not None:
                self._display_segment_for_weld(path_b, seg_b)
                added_by_side[c["side_b"]] += 1

        self.active_weld_id = path_a["id"]
        self._sync_collected_segments()
        self._auto_order_segments_for_continuity(path_a["segments"])
        self._auto_order_segments_for_continuity(path_b["segments"])
        self._reassign_segment_colors()

        ctx = self.viewer._display.GetContext()
        if self.face_a is not None:
            ctx.Remove(self.face_a[2], False)
            self.face_a = None
        if self.face_b is not None:
            ctx.Remove(self.face_b[2], False)
            self.face_b = None
        self.viewer._display.Repaint()

        self.lbl_path.setText(
            f"Gap paths added:\n"
            f"  Path {side_a}: {added_by_side[side_a]} segment(s)\n"
            f"  Path {side_b}: {added_by_side[side_b]} segment(s)"
        )
        self._refresh_weld_list()
        self._rebuild_segment_list()
        self._update_segments_label()
        self._update_status()
        self._update_status_bar()
        print(f"[ALGO] Gap edge pairs applied: Path {side_a}={added_by_side[side_a]}, "
              f"Path {side_b}={added_by_side[side_b]}")
        added = sum(added_by_side.values())
        skipped = 0

        if skipped > 0 and added == 0:
            QMessageBox.information(
                self, "All Already Added",
                f"Seçilen tüm segmentler zaten daha önce eklenmişti.\n"
                f"Skipped: {skipped}\n\n"
                f"Aynı segmenti tekrar eklemek için önce listeden silin."
            )
        elif skipped > 0:
            QMessageBox.information(
                self, "Some Duplicates Skipped",
                f"Eklenen: {added}\n"
                f"Atlanan (duplicate): {skipped}"
            )


    # ─── Wire builder ──────────────────────────────────────────────────

    def _build_wires(self, edges_info):
        if not edges_info:
            return []
        if len(edges_info) == 1:
            edge, length = edges_info[0]
            try:
                mk = BRepBuilderAPI_MakeWire(edge)
                if not mk.IsDone():
                    return []
                wire = mk.Wire()
                return [{
                    "wire": wire, "length": length,
                    "closed": BRep_Tool.IsClosed(wire), "n_edges": 1,
                }]
            except Exception as e:
                print(f"[ERROR] Single-edge wire build: {e}")
                return []

        seq = TopTools_HSequenceOfShape()
        for edge, _ in edges_info:
            seq.Append(edge)
        wire_seq = TopTools_HSequenceOfShape()
        ShapeAnalysis_FreeBounds.ConnectEdgesToWires(
            seq, WIRE_TOLERANCE, False, wire_seq
        )
        wires = []
        for i in range(1, wire_seq.Length() + 1):
            wire = wire_seq.Value(i)
            length = _edge_length(wire)
            if length < EDGE_MIN_LENGTH:
                continue
            n_edges = 0
            ex = TopExp_Explorer(wire, TopAbs_EDGE)
            while ex.More():
                n_edges += 1
                ex.Next()
            wires.append({
                "wire": wire, "length": length,
                "closed": BRep_Tool.IsClosed(wire), "n_edges": n_edges,
            })
        wires.sort(key=lambda w: w["length"], reverse=True)
        return wires

    def _path_type_label(self, w):
        if w["closed"]:
            return "Closed loop"
        if w["n_edges"] == 1:
            return "Arc / single segment"
        return "Open path"

    def _method_human_label(self, method):
        """Internal method name → operatör için anlaşılır kısa etiket."""
        if method == "Topological shared":
            return "Shared edge"
        if method == "Geometric coincident":
            return "Touching edge"
        if method == "Section":
            return "Surface intersection"
        if method.startswith("Bridging-"):
            side = method.split()[0].split("-", 1)[1]
            return f"Path {side}"
        if method == "Manual":
            return "Manual selection"
        if method == "Point contact":
            return "Point contact"
        return method

    def _method_context(self, method):
        if method == "Topological shared":
            return "topological"
        if method == "Geometric coincident":
            return "geometric"
        if method == "Section":
            return "section"
        if method.startswith("Bridging-"):
            return "bridging"
        if method == "Manual":
            return "manual"
        if method == "Point contact":
            return "point_contact"
        return "unknown"

    def _path_letter_for_body(self, body_idx):
        if body_idx is None or body_idx <= 0:
            return "?"
        letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        if body_idx <= len(letters):
            return letters[body_idx - 1]
        return f"B{body_idx}"


    # ─── Segment colors ───────────────────────────────────────────────

    def _segment_color(self, idx):
        r, g, b = SEGMENT_PALETTE[idx % len(SEGMENT_PALETTE)]
        return Quantity_Color(r, g, b, Quantity_TOC_RGB)

    def _segment_color_qt(self, idx):
        r, g, b = SEGMENT_PALETTE[idx % len(SEGMENT_PALETTE)]
        return QColor(int(r * 255), int(g * 255), int(b * 255))

    def _reassign_segment_colors(self):
        ctx = self.viewer._display.GetContext()
        for weld in self.welds:
            is_active = weld["id"] == self.active_weld_id
            for i, seg in enumerate(weld["segments"]):
                color = self._segment_color(i) if is_active else Quantity_Color(
                    0.45, 0.45, 0.45, Quantity_TOC_RGB
                )
                width = 5.0 if is_active else 3.0
                for ais in seg.get("ais_list", []):
                    ctx.SetColor(ais, color, True)
                    ctx.SetWidth(ais, width, True)
        self.viewer._display.Repaint()

    def _show_no_shared_edge_dialog(self, gap):
        QMessageBox.warning(
            self, "No Shared Edge",
            "Seçilen iki yüz birbirine değiyor (touching) ancak ortak bir kenar paylaşmıyor.\n\n"
            "Bu genellikle şu durumlarda olur:\n"
            "  • Yüzler sadece bir köşede temas ediyor (kenar boyunca değil)\n"
            "  • Yanlış yüzler seçilmiş — birbirini bir hat boyunca takip eden yüzler seçin\n\n"
            "Çözüm:\n"
            "  • Daha bariz şekilde bir kenar boyunca buluşan yüzler seçin\n"
            "  • Manual mode'a geçin (kenarı doğrudan tıklayın)"
        )

    def _show_gap_faces_not_facing_dialog(self, gap):
        gap_text = "hesaplanamadı" if gap is None else f"{gap:.2f} mm"
        QMessageBox.information(
            self, "Faces Not Opposing",
            f"Seçilen iki yüz arasında {gap_text} boşluk var, ancak yüzler birbirine bakmıyor.\n\n"
            "Otomatik gap path yalnızca karşılıklı bakan face çiftleri için çalıştırılır.\n\n"
            "Çözüm:\n"
            "  • Karşılıklı bakan iki face seçin\n"
            "  • Veya Manual mode'a geçip kaynak kenarını doğrudan seçin"
        )

    def _ask_add_point_contact_segment(self, point):
        x, y, z = point
        return QMessageBox.question(
            self, "Point Contact Detected",
            "Seçilen iki yüzde ortak kenar bulunamadı; temas sadece tek bir noktada görünüyor.\n\n"
            f"Nokta: ({x:.3f}, {y:.3f}, {z:.3f}) mm\n\n"
            "Bu temas noktasını 0.0 mm uzunlukta noktasal bir path olarak eklemek ister misiniz?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        ) == QMessageBox.Yes

    def _add_point_contact_segment(self, point):
        segments = self._active_segments()
        candidate = {
            "wires": [], "ais_list": [],
            "method": "Point contact", "length": 0.0, "type": "Point contact",
            "start_point": point, "end_point": point, "is_closed": False,
        }
        dup_idx = self._is_duplicate_segment(candidate)
        if dup_idx is not None:
            answer = QMessageBox.question(
                self, "Duplicate Point Contact",
                f"Bu noktasal path Segment {dup_idx} ile aynı konumda görünüyor.\n"
                f"Yine de eklensin mi?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            if answer != QMessageBox.Yes:
                self.reset_face_selection()
                return

        vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(point[0], point[1], point[2])).Vertex()
        color = self._segment_color(len(segments))
        ctx = self.viewer._display.GetContext()
        ais = AIS_Shape(vertex)
        ais.SetColor(color)
        ais.SetWidth(8.0)
        ctx.Display(ais, False)
        self.viewer._display.Repaint()

        candidate["ais_list"] = [ais]
        segments.append(candidate)
        active = self._active_weld()
        if active is not None:
            active["context"] = "point_contact"

        if self.face_a is not None:
            ctx.Remove(self.face_a[2], False)
            self.face_a = None
        if self.face_b is not None:
            ctx.Remove(self.face_b[2], False)
            self.face_b = None
        self.viewer._display.Repaint()

        self.lbl_path.setText(
            "Point contact segment:\n"
            f"  Point:  ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) mm\n"
            "  Length: 0.00 mm"
        )
        print(f"[RESULT] Point contact segment added at {point}")
        self._auto_order_active_weld()
        self._rebuild_segment_list()
        self._refresh_weld_list()
        self._update_segments_label()
        self._update_status()
        self._update_status_bar()

    def _show_faces_too_far_dialog(self, gap):
        gap_text = "hesaplanamadı" if gap is None else f"{gap:.2f} mm"
        QMessageBox.warning(
            self, "Faces Too Far Apart",
            f"Seçilen iki yüz {gap_text} uzaklıkta — kaynak için çok uzak.\n\n"
            "Çözüm:\n"
            "  • Birbirine yakın (touching veya <5mm) yüzler seçin\n"
            "  • Modeli kontrol edin: doğru parçalar mı?"
        )

    def _try_bridging_flow(self, face_a, face_b, body_a, body_b, gap):
        candidates = self._find_gap_edge_pairs(face_a, face_b)
        print(f"[ALGO] Gap edge pairs found: {len(candidates)}")

        if candidates:
            dialog = GapEdgePairsDialog(candidates, parent=self)
            if dialog.exec_() == QDialog.Accepted:
                selected = dialog.selected_indices()
                if selected:
                    self._apply_gap_edge_pairs(candidates, selected)
            return

        QMessageBox.information(
            self, "Gap Detected - Manual Selection Suggested",
            f"Secilen iki yuz arasinda {gap:.2f} mm bosluk var ve yuzler birbirine bakiyor, "
            "ancak otomatik olarak uygun bir edge cifti bulunamadi.\n\n"
            "Bu geometri icin Manual mode'a gecip kaynak kenarini dogrudan secin."
        )

    # ─── Add Segment ──────────────────────────────────────────────────

    def add_segment(self):
        if self.face_a is None or self.face_b is None:
            return

        if self.active_weld_id is None:
            self._create_new_weld(name=None)

        gap, warning = self._validate_face_proximity()
        face_a = self.face_a[0]
        face_b = self.face_b[0]
        body_a = self.face_a[1]
        body_b = self.face_b[1]

        if warning:
            self.lbl_proximity.setText(warning)
        else:
            contact = (
                "Touching" if gap is not None and gap < 1e-3
                else f"Gap: {gap:.3f} mm" if gap is not None else ""
            )
            self.lbl_proximity.setText(contact)

        print(f"\n=== ADD SEGMENT ===  Body {body_a} ↔ Body {body_b}  "
              f"[gap={gap}]")

        edges_info = self._find_shared_edges(face_a, face_b)
        method_used = "Topological shared" if edges_info else None
        print(f"[ALGO] Topological shared edge search: {len(edges_info)} edge(s) found")

        if not edges_info:
            edges_info = self._find_coincident_edges(face_a, face_b)
            print(f"[ALGO] Geometric coincident edge search: {len(edges_info)} edge(s) found")
            if edges_info:
                method_used = "Geometric coincident"

        if not edges_info:
            edges_info = self._section_fallback(face_a, face_b)
            print(f"[ALGO] Section fallback: {len(edges_info)} edge(s) found")
            if edges_info:
                method_used = "Section"

        if not edges_info:
            print(f"[ALGO] All 3 algorithms returned 0 edges; gap={gap}")

            TOUCHING_TOL = 1e-3  # 1 mikron — bunun altı "touching" sayılır

            if gap is None:
                self._show_no_shared_edge_dialog(gap)
                return

            if gap < TOUCHING_TOL:
                point = self._find_point_contact(face_a, face_b)
                if point is not None and self._ask_add_point_contact_segment(point):
                    self._add_point_contact_segment(point)
                    return
                self._show_no_shared_edge_dialog(gap)
                return

            if gap < CLEARANCE_THRESHOLD_MM:
                if not self._faces_suitable_for_gap_paths(face_a, face_b):
                    print("[ALGO] Gap path skipped: faces are not opposing")
                    self._show_gap_faces_not_facing_dialog(gap)
                    return
                self._try_bridging_flow(face_a, face_b, body_a, body_b, gap)
                return

            self._show_faces_too_far_dialog(gap)
            return

        wires = self._build_wires(edges_info)
        if not wires:
            QMessageBox.warning(
                self, "Wire Build Failed",
                f"{len(edges_info)} edge bulundu ama wire kurulamadı.\n"
                f"Manual mode'a geçmeyi deneyin."
            )
            return

        primary = wires[0]
        start_pt, end_pt, is_closed = _wire_endpoints(primary["wire"])
        total_length = sum(w["length"] for w in wires)
        ptype = self._path_type_label(primary)
        print(f"[RESULT] Method used: {method_used}, total length: {total_length:.2f} mm")
        for i, w in enumerate(wires, 1):
            print(f"  Wire {i}: {w['length']:.2f} mm  "
                  f"{'[closed]' if w['closed'] else '[open]'}  {w['n_edges']} edge(s)")

        candidate = {
            "wires": wires, "ais_list": [], "method": method_used,
            "length": total_length, "type": ptype,
            "start_point": start_pt, "end_point": end_pt, "is_closed": is_closed,
        }
        dup_idx = self._is_duplicate_segment(candidate)
        if dup_idx is not None:
            answer = QMessageBox.question(
                self, "Duplicate Segment",
                f"Bu segment Segment {dup_idx} ile büyük ölçüde örtüşüyor.\n"
                f"Yine de eklensin mi?",
                QMessageBox.Yes | QMessageBox.No
            )
            if answer != QMessageBox.Yes:
                self.reset_face_selection()
                return

        segments = self._active_segments()
        seg_idx = len(segments)
        color = self._segment_color(seg_idx)
        ctx = self.viewer._display.GetContext()
        segment_ais = []
        for w in wires:
            ais = AIS_Shape(w["wire"])
            ais.SetColor(color)
            ais.SetWidth(5.0)
            ctx.Display(ais, False)
            segment_ais.append(ais)
        self.viewer._display.Repaint()

        candidate["ais_list"] = segment_ais
        segments.append(candidate)
        active = self._active_weld()
        if active is not None:
            active["context"] = self._method_context(method_used)

        if self.face_a is not None:
            ctx.Remove(self.face_a[2], False)
            self.face_a = None
        if self.face_b is not None:
            ctx.Remove(self.face_b[2], False)
            self.face_b = None
        self.viewer._display.Repaint()

        path_text = (
            f"Last segment added:\n"
            f"  Type:   {ptype}\n"
            f"  Length: {total_length:.2f} mm\n"
            f"  Method: {self._method_human_label(method_used)}"
        )
        if method_used == "Section":
            path_text = "⚠ Used Section fallback — verify path visually\n\n" + path_text
        self.lbl_path.setText(path_text)

        self._auto_order_active_weld()
        self._rebuild_segment_list()
        self._refresh_weld_list()
        self._update_segments_label()
        self._update_status()
        self._update_status_bar()

        if self.chk_show_details.isChecked():
            QMessageBox.information(
                self, "Segment Added",
                f"Type:   {ptype}\n"
                f"Length: {total_length:.2f} mm\n"
                f"Method: {self._method_human_label(method_used)}\n"
                f"Closed: {is_closed}"
            )

    # ─── Duplicate detection (non-bridging) ───────────────────────────

    def _is_duplicate_segment(self, candidate):
        if candidate["start_point"] is None:
            return None
        for i, seg in enumerate(self._active_segments()):
            if seg.get("start_point") is None:
                continue
            forward = (
                _pnt_dist(candidate["start_point"], seg["start_point"]) < DUPLICATE_TOL and
                _pnt_dist(candidate["end_point"],   seg["end_point"])   < DUPLICATE_TOL
            )
            reverse = (
                _pnt_dist(candidate["start_point"], seg["end_point"])   < DUPLICATE_TOL and
                _pnt_dist(candidate["end_point"],   seg["start_point"]) < DUPLICATE_TOL
            )
            if forward or reverse:
                return i + 1
        return None

    # ─── Continuity check ─────────────────────────────────────────────

    def _segment_gap(self, seg_a, seg_b):
        method_a = seg_a.get("method", "")
        method_b = seg_b.get("method", "")
        if (method_a.startswith("Bridging-") and method_b.startswith("Bridging-")
                and method_a.split()[0] == method_b.split()[0]):
            return 0.0

        if seg_a.get("end_point") is None:
            return float("inf")
        candidates = []
        if seg_b.get("start_point") is not None:
            candidates.append(_pnt_dist(seg_a["end_point"], seg_b["start_point"]))
        if seg_b.get("end_point") is not None:
            candidates.append(_pnt_dist(seg_a["end_point"], seg_b["end_point"]))
        if not candidates:
            return float("inf")
        return min(candidates)

    def _seg_start_end_for_order(self, seg, reversed_order=False):
        start = seg.get("start_point")
        end = seg.get("end_point")
        if reversed_order:
            return end, start
        return start, end

    def _score_order_candidate(self, ordered):
        score = 0.0
        gaps = 0
        gap_values = []
        for i in range(len(ordered) - 1):
            _, prev_end = self._seg_start_end_for_order(*ordered[i])
            next_start, _ = self._seg_start_end_for_order(*ordered[i + 1])
            gap = _pnt_dist(prev_end, next_start)
            gap_values.append(gap)
            if gap >= CONTINUITY_GAP_TOL:
                gaps += 1
            score += gap
        return gaps, score, gap_values

    def _build_order_candidate(self, segments, start_idx, start_reversed):
        remaining = [(i, seg) for i, seg in enumerate(segments) if i != start_idx]
        ordered = [(segments[start_idx], start_reversed)]
        while remaining:
            _, current_end = self._seg_start_end_for_order(*ordered[-1])
            best = None
            for rem_idx, (orig_idx, seg) in enumerate(remaining):
                for reversed_order in (False, True):
                    cand_start, _ = self._seg_start_end_for_order(seg, reversed_order)
                    gap = _pnt_dist(current_end, cand_start)
                    if best is None or gap < best[0]:
                        best = (gap, rem_idx, seg, reversed_order)
            _, rem_idx, seg, reversed_order = best
            ordered.append((seg, reversed_order))
            remaining.pop(rem_idx)
        return ordered

    def _best_continuity_order(self, segments):
        if len(segments) < 2:
            return [(seg, False) for seg in segments], 0, []

        best = None
        for start_idx, seg in enumerate(segments):
            for start_reversed in (False, True):
                ordered = self._build_order_candidate(segments, start_idx, start_reversed)
                gaps, score, gap_values = self._score_order_candidate(ordered)
                if best is None or (gaps, score) < (best[0], best[1]):
                    best = (gaps, score, gap_values, ordered)
        return best[3], best[0], best[2]

    def _apply_segment_orientation(self, seg, reversed_order):
        if not reversed_order:
            seg["reversed_for_continuity"] = False
            return
        seg["start_point"], seg["end_point"] = seg.get("end_point"), seg.get("start_point")
        seg["reversed_for_continuity"] = not seg.get("reversed_for_continuity", False)

    def _auto_order_segments_for_continuity(self, segments):
        if len(segments) < 2:
            return False
        old_order = list(segments)
        ordered, _, _ = self._best_continuity_order(segments)
        new_order = []
        orientation_changed = False
        for seg, reversed_order in ordered:
            if reversed_order:
                orientation_changed = True
            self._apply_segment_orientation(seg, reversed_order)
            new_order.append(seg)
        changed = old_order != new_order or orientation_changed
        if changed:
            segments[:] = new_order
        return changed

    def _auto_order_active_weld(self):
        changed = self._auto_order_segments_for_continuity(self._active_segments())
        if changed:
            self._reassign_segment_colors()
        return changed

    def _continuity_summary(self):
        segments = self._active_segments()
        n = len(segments)
        if n < 2:
            return n, 0, []
        connected = 0
        gaps = 0
        gap_list = []
        for i in range(n - 1):
            gap = self._segment_gap(
                segments[i],
                segments[i + 1]
            )
            if gap < CONTINUITY_GAP_TOL:
                connected += 1
                gap_list.append(None)
            else:
                gaps += 1
                gap_list.append(gap)
        return connected, gaps, gap_list

    # ─── Segment list widget ──────────────────────────────────────────

    def _rebuild_segment_list(self):
        segments = self._active_segments()
        self.segment_list.blockSignals(True)
        self.segment_list.clear()
        _, _, gap_list = self._continuity_summary()
        for i, seg in enumerate(segments):
            method_label = self._method_human_label(seg.get("method", ""))
            text = f"Segment {i + 1} — {seg['length']:.1f} mm — {method_label}"
            item = QListWidgetItem(text)
            item.setForeground(QBrush(self._segment_color_qt(i)))
            method = seg.get("method", "")
            if method == "Section" or method.startswith("Bridging"):
                f = QFont()
                f.setItalic(True)
                f.setBold(True)
                item.setFont(f)
            item.setData(Qt.UserRole, i)
            self.segment_list.addItem(item)

            if i < len(gap_list) and gap_list[i] is not None:
                gap_item = QListWidgetItem(f"   ↓ gap: {gap_list[i]:.1f} mm")
                gap_item.setForeground(QBrush(QColor(150, 150, 150)))
                gf = QFont()
                gf.setItalic(True)
                gap_item.setFont(gf)
                gap_item.setFlags(gap_item.flags() & ~Qt.ItemIsSelectable)
                gap_item.setData(Qt.UserRole, -1)
                self.segment_list.addItem(gap_item)
        self.segment_list.blockSignals(False)
        print(f"[UI] Segment list rebuilt: {len(segments)} segment(s)")

    def _on_segment_selected(self):
        items = self.segment_list.selectedItems()
        if not items:
            self.btn_delete_segment.setEnabled(False)
            self.btn_move_up.setEnabled(False)
            self.btn_move_down.setEnabled(False)
            return
        idx = items[0].data(Qt.UserRole)
        if idx is None or idx == -1:
            self.btn_delete_segment.setEnabled(False)
            self.btn_move_up.setEnabled(False)
            self.btn_move_down.setEnabled(False)
            return
        self.btn_delete_segment.setEnabled(True)
        self.btn_move_up.setEnabled(idx > 0)
        self.btn_move_down.setEnabled(idx < len(self._active_segments()) - 1)
        self._highlight_segment(idx)

    def _highlight_segment(self, highlight_idx):
        ctx = self.viewer._display.GetContext()
        for i, seg in enumerate(self._active_segments()):
            w = 8.0 if i == highlight_idx else 5.0
            for ais in seg["ais_list"]:
                ctx.SetWidth(ais, w, True)
        self.viewer._display.Repaint()

    def _select_segment_in_list(self, idx):
        for row in range(self.segment_list.count()):
            item = self.segment_list.item(row)
            if item.data(Qt.UserRole) == idx:
                self.segment_list.setCurrentItem(item)
                return

    # ─── Delete / Reorder / Undo ──────────────────────────────────────

    def delete_selected_segment(self):
        items = self.segment_list.selectedItems()
        if not items:
            return
        idx = items[0].data(Qt.UserRole)
        segments = self._active_segments()
        if idx is None or idx < 0 or idx >= len(segments):
            return
        ctx = self.viewer._display.GetContext()
        for ais in segments[idx].get("ais_list", []):
            ctx.Remove(ais, False)
        segments.pop(idx)
        self._reassign_segment_colors()
        self.viewer._display.Repaint()
        self._rebuild_segment_list()
        self._refresh_weld_list()
        self._update_segments_label()
        self._update_status_bar()
        print(f"[UI] Deleted segment {idx + 1}")

    def move_segment_up(self):
        items = self.segment_list.selectedItems()
        if not items:
            return
        idx = items[0].data(Qt.UserRole)
        if idx is None or idx <= 0:
            return
        segments = self._active_segments()
        segments[idx], segments[idx - 1] = (
            segments[idx - 1], segments[idx]
        )
        self._reassign_segment_colors()
        self._rebuild_segment_list()
        self._select_segment_in_list(idx - 1)
        print(f"[UI] Moved segment up: {idx + 1} → {idx}")

    def move_segment_down(self):
        items = self.segment_list.selectedItems()
        if not items:
            return
        idx = items[0].data(Qt.UserRole)
        segments = self._active_segments()
        if idx is None or idx >= len(segments) - 1:
            return
        segments[idx], segments[idx + 1] = (
            segments[idx + 1], segments[idx]
        )
        self._reassign_segment_colors()
        self._rebuild_segment_list()
        self._select_segment_in_list(idx + 1)
        print(f"[UI] Moved segment down: {idx + 1} → {idx + 2}")

    def undo_last_segment(self):
        segments = self._active_segments()
        if not segments:
            return
        ctx = self.viewer._display.GetContext()
        seg = segments.pop()
        for ais in seg.get("ais_list", []):
            ctx.Remove(ais, False)
        self.viewer._display.Repaint()
        self._rebuild_segment_list()
        self._refresh_weld_list()
        self._update_segments_label()
        self._update_status_bar()
        print("[UI] Undo: removed last segment")

    # ─── Finish Path ───────────────────────────────────────────────────

    def finish_path(self):
        welds_with_segments = [w for w in self.welds if w["segments"]]
        if not welds_with_segments:
            return
        old_active = self.active_weld_id
        grand_total = 0.0
        lines = ["Segments Finalized", "==================", ""]
        for weld in welds_with_segments:
            self.active_weld_id = weld["id"]
            self._sync_collected_segments()
            self._auto_order_active_weld()
            segments = self._active_segments()
            total = sum(s["length"] for s in segments)
            grand_total += total
            connected, gaps, _ = self._continuity_summary()
            possible_joins = max(0, len(segments) - 1)
            lines.append(f"Path {weld['id']} — {weld['name']}")
            lines.append(f"  Total length: {total:.2f} mm")
            lines.append(
                f"  Continuity: {connected}/{possible_joins} joins connected, {gaps} gaps"
            )
            lines.append("  Segments:")
            for i, seg in enumerate(segments):
                method = self._method_human_label(seg.get("method", ""))
                lines.append(f"    {i + 1}. {method:<24} {seg['length']:>7.2f} mm")
            lines.append("")
        self.active_weld_id = old_active
        self._sync_collected_segments()
        self._reassign_segment_colors()
        self._rebuild_segment_list()
        self._refresh_weld_list()
        lines.append("──────────────────")
        lines.append(f"Total: {len(welds_with_segments)} path(s), {grand_total:.2f} mm")
        msg = "\n".join(lines)
        print(f"\n=== FINISH PATH ===\n{msg}")
        QMessageBox.information(self, "Segments Finalized", msg)

    # ─── Manual apply ──────────────────────────────────────────────────

    def apply_manual(self):
        if not self.manual_edges:
            return
        if self.active_weld_id is None:
            self._create_new_weld(name=None, context="manual")
        edges_info = [(e, _edge_length(e)) for e in self.manual_edges]

        ctx = self.viewer._display.GetContext()
        for ais in self.manual_ais_list:
            ctx.Remove(ais, False)
        self.manual_ais_list.clear()

        wires = self._build_wires(edges_info)
        if not wires:
            QMessageBox.warning(
                self, "Path Error",
                "Could not build a wire from the selected edges."
            )
            return

        primary = wires[0]
        start_pt, end_pt, is_closed = _wire_endpoints(primary["wire"])
        total = sum(w["length"] for w in wires)
        ptype = self._path_type_label(primary)

        segments = self._active_segments()
        seg_idx = len(segments)
        color = self._segment_color(seg_idx)
        segment_ais = []
        for w in wires:
            ais = AIS_Shape(w["wire"])
            ais.SetColor(color)
            ais.SetWidth(5.0)
            ctx.Display(ais, False)
            segment_ais.append(ais)
        self.viewer._display.Repaint()

        segments.append({
            "wires": wires, "ais_list": segment_ais,
            "method": "Manual", "length": total, "type": ptype,
            "start_point": start_pt, "end_point": end_pt, "is_closed": is_closed,
        })
        active = self._active_weld()
        if active is not None:
            active["context"] = "manual"
        self.manual_edges.clear()
        self.btn_apply_manual.setEnabled(False)
        self.lbl_path.setText(
            f"Manual segment:\n  Type:   {ptype}\n  Length: {total:.2f} mm"
        )
        print(f"[RESULT] Manual path: {ptype}, {total:.2f} mm")
        self._auto_order_active_weld()
        self._rebuild_segment_list()
        self._refresh_weld_list()
        self._update_segments_label()
        self._update_status_bar()

    # ─── JSON IO ───────────────────────────────────────────────────────

    def _segments_to_json(self):
        def segment_to_json(i, seg):
            return {
                "index": i + 1,
                "type": seg.get("type", ""),
                "method": seg.get("method", ""),
                "length": seg.get("length", 0.0),
                "start_point": _pnt_to_list(seg.get("start_point")),
                "end_point":   _pnt_to_list(seg.get("end_point")),
                "is_closed":   seg.get("is_closed", False),
                "reversed_for_continuity": seg.get("reversed_for_continuity", False),
            }

        return {
            "version": "5.K",
            "step_file": self.current_step_file,
            "welds": [
                {
                    "id": weld["id"],
                    "name": weld["name"],
                    "context": weld.get("context", ""),
                    "segments": [
                        segment_to_json(i, seg)
                        for i, seg in enumerate(weld["segments"])
                    ],
                }
                for weld in self.welds
            ],
            "total_length": sum(
                s.get("length", 0.0)
                for weld in self.welds
                for s in weld["segments"]
            ),
        }

    def save_path_json(self):
        total_segments = sum(len(w["segments"]) for w in self.welds)
        if total_segments == 0:
            QMessageBox.information(self, "No Path", "No segments to save.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Path As JSON", "", "JSON files (*.json)"
        )
        if not path:
            return
        if not path.lower().endswith(".json"):
            path += ".json"
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self._segments_to_json(), f, indent=2, ensure_ascii=False)
            print(f"[IO] Saved {total_segments} segment(s) to {path}")
            QMessageBox.information(
                self, "Saved",
                f"Saved {len(self.welds)} path(s), {total_segments} segment(s) to:\n{path}"
            )
        except Exception as e:
            print(f"[IO] Save failed: {e}")
            QMessageBox.critical(self, "Save Failed", str(e))

    def load_path_json(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Path From JSON", "", "JSON files (*.json)"
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"[IO] Load failed: {e}")
            QMessageBox.critical(self, "Load Failed", str(e))
            return
        if "welds" in data:
            welds = data.get("welds", [])
        else:
            welds = [{
                "id": 1,
                "name": "Path 1",
                "context": "loaded",
                "segments": data.get("segments", []),
            }]
        n = sum(len(w.get("segments", [])) for w in welds)
        total = data.get(
            "total_length",
            sum(
                s.get("length", 0.0)
                for w in welds
                for s in w.get("segments", [])
            )
        )
        self.welds = []
        for w in welds:
            self.welds.append({
                "id": int(w.get("id", len(self.welds) + 1)),
                "name": w.get("name", f"Path {len(self.welds) + 1}"),
                "segments": [
                    dict(seg, ais_list=seg.get("ais_list", []))
                    for seg in w.get("segments", [])
                ],
                "created_at": time.time(),
                "context": w.get("context", "loaded"),
            })
        self.active_weld_id = self.welds[0]["id"] if self.welds else None
        self._sync_collected_segments()
        self.loaded_metadata = data
        self._refresh_weld_list()
        self._rebuild_segment_list()
        self._update_segments_label()
        self._update_status_bar()
        msg = (
            f"Loaded {n} segment(s) from:\n{path}\n\n"
            f"STEP file: {data.get('step_file', '(unknown)')}\n"
            f"Total length: {total:.2f} mm\n\n"
            f"Loaded path metadata. Reopen the STEP file to recreate geometry."
        )
        print(f"[IO] Loaded metadata: {n} segment(s), total {total:.2f} mm")
        QMessageBox.information(self, "Loaded", msg)

    def export_path_for_trajectory(self):
        total_segments = sum(len(w["segments"]) for w in self.welds)
        if total_segments == 0:
            QMessageBox.information(self, "No Path", "No segments to export.")
            return
        default = "trajectory_export.json"
        if self.current_step_file:
            base = self.current_step_file.rsplit("/", 1)[-1].rsplit("\\", 1)[-1]
            base = base.rsplit(".", 1)[0]
            default = f"{base}_trajectory.json"
        path, _ = QFileDialog.getSaveFileName(
            self, "Export Path for Trajectory Planning", default, "JSON files (*.json)"
        )
        if not path:
            return
        if not path.lower().endswith(".json"):
            path += ".json"
        try:
            data = self._segments_to_json()
            data["export_purpose"] = "trajectory_planning"
            data["note"] = "Faz 4'te WeldPoint listesine genişletilecek."
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            print(f"[IO] Exported trajectory file: {path}")
            QMessageBox.information(
                self, "Exported",
                f"Exported {len(self.welds)} path(s), {total_segments} segment(s) to:\n{path}\n\n"
                f"Faz 4'teki trajectory planner'ın input'u olarak kullanılacak."
            )
        except Exception as e:
            print(f"[IO] Export failed: {e}")
            QMessageBox.critical(self, "Export Failed", str(e))

    # ─── Status ────────────────────────────────────────────────────────

    def _update_status(self):
        if not self.solids:
            self.lbl_status.setText("Open a STEP file to begin.")
            self.btn_add_segment.setEnabled(False)
            self._update_face_labels()
            return
        if self.radio_manual.isChecked():
            self.lbl_status.setText("Manual mode:\nClick edges on the model.")
            self._update_face_labels()
            return

        if self._active_segments() and self.face_a is None and self.face_b is None:
            self.lbl_status.setText(
                "Segment added.\n"
                "Select next face pair to add another segment,\n"
                "or click 'Finalize All Segments' to complete."
            )
        elif self.face_a is None and self.face_b is None:
            self.lbl_status.setText(
                "Auto (Face Pair):\n"
                "1) Click a face on Body A.\n"
                "2) Click a face on Body B.\n"
                "   Selected faces should share a common edge\n"
                "   (i.e. visibly meet along a line)."
            )
        elif self.face_a is not None and self.face_b is None:
            self.lbl_status.setText(
                f"Face X (Body {self.face_a[1]}) selected.\n"
                f"Now click a face on a different body."
            )
        else:
            self.lbl_status.setText("Both faces selected.\nClick 'Add Segment' (or Enter).")

        self._update_face_labels()
        self.btn_add_segment.setEnabled(
            self.face_a is not None and self.face_b is not None
            and self.radio_auto.isChecked()
        )

    def _update_face_labels(self):
        if self.face_a is not None:
            _, b_idx, _, area, c = self.face_a
            self.lbl_face_a.setText(
                f"Face X:  Body {b_idx}  ✓\n"
                f"  Area: {area:.1f} mm²\n"
                f"  Center: ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f})"
            )
        else:
            self.lbl_face_a.setText("Face X:  —")
        if self.face_b is not None:
            _, b_idx, _, area, c = self.face_b
            self.lbl_face_b.setText(
                f"Face Y:  Body {b_idx}  ✓\n"
                f"  Area: {area:.1f} mm²\n"
                f"  Center: ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f})"
            )
        else:
            self.lbl_face_b.setText("Face Y:  —")

    def _update_segments_label(self):
        segments = self._active_segments()
        n = len(segments)
        total = sum(s["length"] for s in segments)
        self.lbl_segments.setText(f"Segments: {n}  |  Total: {total:.1f} mm")
        any_segments = any(w["segments"] for w in self.welds)
        self.btn_finish.setEnabled(any_segments)
        self.btn_clear_segments.setEnabled(n > 0)
        self.btn_undo.setEnabled(n > 0)

    def _update_status_bar(self):
        if not self.solids:
            self.statusBar().showMessage("Ready — open a STEP file to begin")
            return
        fname = self.current_step_file.rsplit("/", 1)[-1].rsplit("\\", 1)[-1]
        n_seg = sum(len(w["segments"]) for w in self.welds)
        total = sum(s["length"] for w in self.welds for s in w["segments"])
        self.statusBar().showMessage(
            f"File: {fname}  |  Bodies: {len(self.solids)}  |  "
            f"Paths: {len(self.welds)}  |  Segments: {n_seg}  |  Total: {total:.1f} mm"
        )


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    window.init_viewer()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
