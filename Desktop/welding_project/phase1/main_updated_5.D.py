"""
Robotic Laser Welding — Phase 3   (main_updated_5.D.py)
========================================================
5.B → 5.C eklenenler:
  • Segment list widget (QListWidget) — tek tek delete, reorder, click-to-highlight
  • Move Up / Move Down — segmentleri yeniden sırala
  • Undo Last Segment — son eklenen segmenti geri al (Ctrl+Z)
  • "Show details after each segment" checkbox
  • Segment endpoints (start_point, end_point, is_closed) — Faz 4 hazırlığı
  • Proximity check artık engelleyici değil (sadece uyarı)
  • Section method için görsel uyarı (italic font + ⚠ prefix)
  • Segment continuity check — ardışık segmentler arası gap detection
  • Duplicate segment detection — aynı edge'i yeniden ekleme uyarısı
  • Status bar — File / Bodies / Segments / Total
  • Keyboard shortcuts — Ctrl+O/S/L/E/Z, Enter, Escape, Delete
  • Face info (area, center) — Face A/B etiketlerinde
  • Color legend
  • JSON Save / Load / Export — Faz 4 trajectory planning köprüsü
  • [ALGO]/[PICK]/[RESULT]/[STATE]/[FILE]/[UI]/[IO] log prefix'leri

5.C → 5.D eklenenler:
  • Clearance fit detection: gap < 5mm ise plug/fillet welding ipucu
  • Algorithm 4: Closest-edge bridging (operatör onayıyla)
  • Bridging segment'ler liste'de italic+bold (görsel doğrulama uyarısı)
  • Path label'da "⚠ verify path visually" uyarısı

Algoritma sırası (5.D):
  1) Topological shared-edge (IsSame, TShape pointer)
  2) Geometric coincident-edge (length + endpoint + dist)
  3) Section fallback
  4) Clearance check + Bridging proposal (operatör onayıyla)
  5) Hata: Manual mode öner

Operatör akışı:
  1) Auto mode'da iki face seç (ortak kenarı olan)
  2) "Add Segment" (veya Enter) → segment biriktirilir
  3) Yeni face çifti → yeni segment
  4) Liste'de delete/reorder/undo
  5) "Finish Path" → özet
  6) "Save Path As JSON" → Faz 4'e geç
"""

import sys
import json

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QAction,
    QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QFrame,
    QMessageBox, QRadioButton, QGroupBox, QListWidget, QListWidgetItem,
    QShortcut, QCheckBox
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
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopTools import TopTools_HSequenceOfShape
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeWire
from OCC.Core.BRepTools import BRepTools_WireExplorer
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.Core.BRep import BRep_Tool


# ── Parametreler ────────────────────────────────────────────────────────────
PROXIMITY_THRESHOLD_MM   = 5.0
EDGE_MIN_LENGTH          = 0.01
WIRE_TOLERANCE           = 0.5
COINCIDENT_LEN_TOL       = 0.01
COINCIDENT_ENDPOINT_TOL  = 0.1
COINCIDENT_DIST_TOL      = 0.1
CONTINUITY_GAP_TOL       = 0.5
DUPLICATE_TOL            = 0.5

# 5.D — Clearance fit / bridging
CLEARANCE_THRESHOLD_MM   = 5.0    # bu altında clearance fit önerilir
BRIDGING_MIN_DIST_MM     = 0.05   # bu altı zaten Algo 2'de yakalanır
BRIDGING_MAX_DIST_MM     = 5.0    # bu üstü "köprü" sayılmaz
BRIDGING_LEN_TOL         = 0.5    # iki edge'in uzunluk farkı toleransı (mm)

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
    """
    5.D eklenenler: clearance fit detection + closest-edge bridging.
    """

    COLOR_FACE_A   = Quantity_Color(1.00, 0.85, 0.00, Quantity_TOC_RGB)
    COLOR_FACE_B   = Quantity_Color(0.10, 0.70, 1.00, Quantity_TOC_RGB)
    COLOR_CYAN     = Quantity_Color(0.00, 0.85, 1.00, Quantity_TOC_RGB)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Laser Welding – Phase 3 (Bridging Aware)")
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

    # ─── Side panel ────────────────────────────────────────────────────

    def _create_side_panel(self):
        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        panel.setMinimumWidth(320)
        v = QVBoxLayout()

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

        self.lbl_face_a = QLabel("Face A:  —")
        self.lbl_face_a.setWordWrap(True)
        self.lbl_face_a.setStyleSheet(
            "padding: 4px; color: #b08000; font-weight: bold;"
        )
        v.addWidget(self.lbl_face_a)

        self.lbl_face_b = QLabel("Face B:  —")
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
        self.btn_reset_faces.setToolTip("Clear face A/B (Esc)")
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
        self.lbl_legend = QLabel("Colors: red→orange→yellow")
        self.lbl_legend.setStyleSheet("padding: 4px; color: #888; font-style: italic;")
        seg_header_layout.addWidget(self.lbl_legend)
        v.addLayout(seg_header_layout)

        self.segment_list = QListWidget()
        self.segment_list.setMinimumHeight(140)
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

        self.btn_finish = QPushButton("Finish Path")
        self.btn_finish.clicked.connect(self.finish_path)
        self.btn_finish.setEnabled(False)
        v.addWidget(self.btn_finish)

        self.btn_clear_segments = QPushButton("Clear All Segments")
        self.btn_clear_segments.clicked.connect(self.clear_all_segments)
        self.btn_clear_segments.setEnabled(False)
        v.addWidget(self.btn_clear_segments)

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
        return panel

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
                      self.lbl_segments, self.lbl_legend, self.segment_list):
                w.setVisible(True)
            self.btn_clear_edges.setVisible(False)
            self.btn_apply_manual.setVisible(False)
        else:
            self._activate_selection_mode(SEL_MODE_EDGE)
            for w in (self.btn_add_segment, self.btn_finish, self.btn_undo,
                      self.btn_clear_segments, self.btn_delete_segment,
                      self.btn_move_up, self.btn_move_down,
                      self.lbl_face_a, self.lbl_face_b, self.lbl_segments,
                      self.lbl_legend, self.segment_list):
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
        self.collected_segments.clear()
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
                    f"Bu face Body {body_idx}'den; Face A da aynı body'den.\n"
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
                    f"Bu face Body {body_idx}'den; Face A da aynı body'den.\n"
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
        print(f"[PICK] Face A → Body {body_idx}, area={area:.2f} mm², center={center}")

    def _set_face_b(self, face, body_idx):
        area, center = _face_info(face)
        ais = self._highlight_face(face, self.COLOR_FACE_B)
        self.face_b = (face, body_idx, ais, area, center)
        print(f"[PICK] Face B → Body {body_idx}, area={area:.2f} mm², center={center}")

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
        for seg in self.collected_segments:
            for ais in seg["ais_list"]:
                ctx.Remove(ais, False)
        self.collected_segments.clear()
        self.viewer._display.Repaint()
        self.lbl_path.setText("")
        self._rebuild_segment_list()
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

    # ─── Validation (warning, not blocking) ───────────────────────────

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

    # ─── ALGO 4: Closest-edge bridging (5.D yenisi) ───────────────────

    def _try_closest_edge_bridging(self, face_a, face_b):
        """
        Clearance fit senaryosunda kullanılan 4. algoritma.
        Face A'nın her edge'i ile Face B'nin her edge'i arası min mesafeyi
        hesapla. En yakın çift, BRIDGING_MIN_DIST_MM ≤ d ≤ BRIDGING_MAX_DIST_MM
        VE uzunluk farkı ≤ BRIDGING_LEN_TOL ise döndür.

        Return: (edge_a_info, edge_b_info, mid_dist) veya None
                edge_a_info = (edge, length)
        """
        edges_a = []
        exp = TopExp_Explorer(face_a, TopAbs_EDGE)
        while exp.More():
            e = exp.Current()
            length = _edge_length(e)
            if length >= EDGE_MIN_LENGTH:
                edges_a.append((e, length))
            exp.Next()

        edges_b = []
        exp = TopExp_Explorer(face_b, TopAbs_EDGE)
        while exp.More():
            e = exp.Current()
            length = _edge_length(e)
            if length >= EDGE_MIN_LENGTH:
                edges_b.append((e, length))
            exp.Next()

        if not edges_a or not edges_b:
            return None

        best = None  # (edge_a_info, edge_b_info, dist)
        for edge_a, length_a in edges_a:
            for edge_b, length_b in edges_b:
                if abs(length_a - length_b) > BRIDGING_LEN_TOL:
                    continue
                d = BRepExtrema_DistShapeShape(edge_a, edge_b)
                d.Perform()
                if not d.IsDone():
                    continue
                dist = d.Value()
                if dist < BRIDGING_MIN_DIST_MM:
                    continue  # Algo 2 zaten yakalardı
                if dist > BRIDGING_MAX_DIST_MM:
                    continue
                if best is None or dist < best[2]:
                    best = ((edge_a, length_a), (edge_b, length_b), dist)

        if best is None:
            return None

        print(f"[ALGO] Closest-edge bridging candidate: "
              f"len_a={best[0][1]:.2f}, len_b={best[1][1]:.2f}, "
              f"dist={best[2]:.2f} mm")
        return best

    def _apply_bridging(self, edge_a_info, edge_b_info, mid_dist):
        """
        Operatör onayıyla closest-edge bridging segmenti ekle.
        Body A (genelde levha) edge'ini kullanırız.
        """
        edge, length = edge_a_info
        print(f"[ALGO] Bridging: using Body A edge, length={length:.2f} mm, "
              f"avg gap to opposite={mid_dist:.2f} mm")

        wires = self._build_wires([(edge, length)])
        if not wires:
            QMessageBox.warning(
                self, "Bridging Failed",
                "Köprü adayı edge wire'a dönüştürülemedi."
            )
            return

        primary = wires[0]
        start_pt, end_pt, is_closed = _wire_endpoints(primary["wire"])
        total_length = sum(w["length"] for w in wires)
        ptype = self._path_type_label(primary)
        method_used = f"Bridging (gap {mid_dist:.2f} mm)"

        seg_idx = len(self.collected_segments)
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

        self.collected_segments.append({
            "wires": wires,
            "ais_list": segment_ais,
            "method": method_used,
            "length": total_length,
            "type": ptype,
            "start_point": start_pt,
            "end_point": end_pt,
            "is_closed": is_closed,
        })

        if self.face_a is not None:
            ctx.Remove(self.face_a[2], False)
            self.face_a = None
        if self.face_b is not None:
            ctx.Remove(self.face_b[2], False)
            self.face_b = None
        self.viewer._display.Repaint()

        self.lbl_path.setText(
            f"⚠ Bridging segment added — verify path visually\n\n"
            f"  Type:   {ptype}\n"
            f"  Length: {total_length:.2f} mm\n"
            f"  Method: {method_used}"
        )

        self._rebuild_segment_list()
        self._update_segments_label()
        self._update_status()
        self._update_status_bar()
        print(f"[RESULT] Bridging segment added: {ptype}, {total_length:.2f} mm")

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

    def _segment_color(self, idx):
        g = min(0.85, idx * 0.18)
        return Quantity_Color(1.0, g, 0.0, Quantity_TOC_RGB)

    def _segment_color_qt(self, idx):
        g = min(0.85, idx * 0.18)
        return QColor(255, int(g * 255), 0)

    def _reassign_segment_colors(self):
        ctx = self.viewer._display.GetContext()
        for i, seg in enumerate(self.collected_segments):
            color = self._segment_color(i)
            for ais in seg["ais_list"]:
                ctx.SetColor(ais, color, True)
        self.viewer._display.Repaint()

    # ─── Add Segment (5.D: clearance + bridging eklendi) ──────────────

    def add_segment(self):
        if self.face_a is None or self.face_b is None:
            return

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

        # 1) Topological
        edges_info = self._find_shared_edges(face_a, face_b)
        method_used = "Topological shared" if edges_info else None
        print(f"[ALGO] Topological shared edge search: {len(edges_info)} edge(s) found")

        # 2) Geometric coincident
        if not edges_info:
            edges_info = self._find_coincident_edges(face_a, face_b)
            print(f"[ALGO] Geometric coincident edge search: {len(edges_info)} edge(s) found")
            if edges_info:
                method_used = "Geometric coincident"

        # 3) Section
        if not edges_info:
            edges_info = self._section_fallback(face_a, face_b)
            print(f"[ALGO] Section fallback: {len(edges_info)} edge(s) found")
            if edges_info:
                method_used = "Section"

        # 4) Hâlâ boş — clearance fit kontrolü + bridging önerisi
        if not edges_info:
            print(f"[ALGO] All 3 algorithms returned 0 edges; gap={gap}")

            if gap is not None and gap < CLEARANCE_THRESHOLD_MM:
                print("[ALGO] Trying clearance-fit bridging...")
                bridging_result = self._try_closest_edge_bridging(face_a, face_b)

                msg = (
                    f"Bu iki face arasında paylaşılan bir kenar bulunamadı.\n\n"
                    f"Gap: {gap:.2f} mm — yüzeyler yakın ama temas etmiyor.\n"
                    f"Bu durum bir clearance fit (boşluklu geçirme) olabilir.\n\n"
                    f"Bu tip geometrilerde tipik çözüm:\n"
                    f"  • Manual mode'a geçin\n"
                    f"  • Levhadaki delik kenarındaki daireyi doğrudan tıklayın\n"
                    f"  • Bu, plug/fillet welding için tipik kaynak yoludur"
                )

                if bridging_result is not None:
                    edge_a_info, edge_b_info, mid_dist = bridging_result
                    msg += (
                        f"\n\n──────────\n"
                        f"Otomatik öneri:\n"
                        f"İki face arasında birbirine yakın iki edge tespit ettim:\n"
                        f"  • Body 1 edge: {edge_a_info[1]:.2f} mm\n"
                        f"  • Body 2 edge: {edge_b_info[1]:.2f} mm\n"
                        f"  • Aralarındaki ortalama mesafe: {mid_dist:.2f} mm\n\n"
                        f"Bunlardan birini kaynak yolu olarak ekleyebilirim.\n"
                        f"Kabul ediyor musunuz?"
                    )
                    answer = QMessageBox.question(
                        self, "Clearance Fit Detected",
                        msg,
                        QMessageBox.Yes | QMessageBox.No,
                        QMessageBox.No
                    )
                    if answer == QMessageBox.Yes:
                        self._apply_bridging(edge_a_info, edge_b_info, mid_dist)
                        return
                    else:
                        return
                else:
                    QMessageBox.information(self, "Clearance Fit?", msg)
                    return
            else:
                QMessageBox.warning(
                    self, "No Shared Edge",
                    "Bu iki face arasında paylaşılan bir kenar bulunamadı.\n\n"
                    "Olası nedenler:\n"
                    "  • Seçilen face'ler birbirine değmiyor\n"
                    "    (sadece köşede temas ediyor olabilir)\n"
                    "  • Face'ler aynı body'nin parçası\n"
                    "  • Geometride numerik problem var\n\n"
                    "Çözümler:\n"
                    "  • Daha bariz şekilde bir hat boyunca buluşan\n"
                    "    face'ler seçin\n"
                    "  • Manual mode'a geçin (kenarı doğrudan tıklayın)"
                )
                return

        # Edge'ler bulundu → wire kur
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

        seg_idx = len(self.collected_segments)
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
        self.collected_segments.append(candidate)

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
            f"  Method: {method_used}"
        )
        if method_used == "Section":
            path_text = "⚠ Used Section fallback — verify path visually\n\n" + path_text
        self.lbl_path.setText(path_text)

        self._rebuild_segment_list()
        self._update_segments_label()
        self._update_status()
        self._update_status_bar()

        if self.chk_show_details.isChecked():
            QMessageBox.information(
                self, "Segment Added",
                f"Type:   {ptype}\n"
                f"Length: {total_length:.2f} mm\n"
                f"Method: {method_used}\n"
                f"Closed: {is_closed}"
            )

    # ─── Duplicate detection ──────────────────────────────────────────

    def _is_duplicate_segment(self, candidate):
        if candidate["start_point"] is None:
            return None
        for i, seg in enumerate(self.collected_segments):
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

    def _continuity_summary(self):
        n = len(self.collected_segments)
        if n < 2:
            return n, 0, []
        connected = 0
        gaps = 0
        gap_list = []
        for i in range(n - 1):
            gap = self._segment_gap(
                self.collected_segments[i],
                self.collected_segments[i + 1]
            )
            if gap < CONTINUITY_GAP_TOL:
                connected += 1
                gap_list.append(None)
            else:
                gaps += 1
                gap_list.append(gap)
        return connected, gaps, gap_list

    # ─── Segment list widget (5.D: bridging de italic+bold) ───────────

    def _rebuild_segment_list(self):
        self.segment_list.blockSignals(True)
        self.segment_list.clear()
        _, _, gap_list = self._continuity_summary()
        for i, seg in enumerate(self.collected_segments):
            text = f"Segment {i + 1} — {seg['length']:.1f} mm — {seg['method']}"
            item = QListWidgetItem(text)
            item.setForeground(QBrush(self._segment_color_qt(i)))
            method = seg.get("method", "")
            # Section veya Bridging → italic+bold (görsel doğrulama uyarısı)
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
        print(f"[UI] Segment list rebuilt: {len(self.collected_segments)} segment(s)")

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
        self.btn_move_down.setEnabled(idx < len(self.collected_segments) - 1)
        self._highlight_segment(idx)

    def _highlight_segment(self, highlight_idx):
        ctx = self.viewer._display.GetContext()
        for i, seg in enumerate(self.collected_segments):
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
        if idx is None or idx < 0 or idx >= len(self.collected_segments):
            return
        ctx = self.viewer._display.GetContext()
        for ais in self.collected_segments[idx]["ais_list"]:
            ctx.Remove(ais, False)
        self.collected_segments.pop(idx)
        self._reassign_segment_colors()
        self.viewer._display.Repaint()
        self._rebuild_segment_list()
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
        self.collected_segments[idx], self.collected_segments[idx - 1] = (
            self.collected_segments[idx - 1], self.collected_segments[idx]
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
        if idx is None or idx >= len(self.collected_segments) - 1:
            return
        self.collected_segments[idx], self.collected_segments[idx + 1] = (
            self.collected_segments[idx + 1], self.collected_segments[idx]
        )
        self._reassign_segment_colors()
        self._rebuild_segment_list()
        self._select_segment_in_list(idx + 1)
        print(f"[UI] Moved segment down: {idx + 1} → {idx + 2}")

    def undo_last_segment(self):
        if not self.collected_segments:
            return
        ctx = self.viewer._display.GetContext()
        seg = self.collected_segments.pop()
        for ais in seg["ais_list"]:
            ctx.Remove(ais, False)
        self.viewer._display.Repaint()
        self._rebuild_segment_list()
        self._update_segments_label()
        self._update_status_bar()
        print("[UI] Undo: removed last segment")

    # ─── Finish Path ───────────────────────────────────────────────────

    def finish_path(self):
        if not self.collected_segments:
            return
        n = len(self.collected_segments)
        total = sum(s["length"] for s in self.collected_segments)
        connected, gaps, gap_list = self._continuity_summary()

        lines = [
            f"Path finalized: {n} segment(s)",
            f"Total length:   {total:.2f} mm",
            f"Continuity:     {connected} connected, {gaps} gap(s) detected",
            "",
        ]
        for i, seg in enumerate(self.collected_segments):
            method = seg["method"]
            mark = " *" if (method == "Section" or method.startswith("Bridging")) else ""
            lines.append(f"  {i + 1}. {seg['type']:<22} {seg['length']:>7.2f} mm  ({method}){mark}")
            if i < n - 1:
                if gap_list[i] is None:
                    lines.append("     ↓ connected")
                else:
                    lines.append(f"     ↓ gap: {gap_list[i]:.2f} mm")
        if any(s["method"] == "Section" or s["method"].startswith("Bridging")
               for s in self.collected_segments):
            lines.append("")
            lines.append("  *  Section/Bridging fallback used — verify visually")
        msg = "\n".join(lines)
        print(f"\n=== FINISH PATH ===\n{msg}")
        QMessageBox.information(self, "Path Finalized", msg)

    # ─── Manual apply ──────────────────────────────────────────────────

    def apply_manual(self):
        if not self.manual_edges:
            return
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

        seg_idx = len(self.collected_segments)
        color = self._segment_color(seg_idx)
        segment_ais = []
        for w in wires:
            ais = AIS_Shape(w["wire"])
            ais.SetColor(color)
            ais.SetWidth(5.0)
            ctx.Display(ais, False)
            segment_ais.append(ais)
        self.viewer._display.Repaint()

        self.collected_segments.append({
            "wires": wires, "ais_list": segment_ais,
            "method": "Manual", "length": total, "type": ptype,
            "start_point": start_pt, "end_point": end_pt, "is_closed": is_closed,
        })
        self.manual_edges.clear()
        self.btn_apply_manual.setEnabled(False)
        self.lbl_path.setText(
            f"Manual segment:\n  Type:   {ptype}\n  Length: {total:.2f} mm"
        )
        print(f"[RESULT] Manual path: {ptype}, {total:.2f} mm")
        self._rebuild_segment_list()
        self._update_segments_label()
        self._update_status_bar()

    # ─── JSON IO ───────────────────────────────────────────────────────

    def _segments_to_json(self):
        return {
            "version": "5.D",
            "step_file": self.current_step_file,
            "segments": [
                {
                    "index": i + 1,
                    "type": seg.get("type", ""),
                    "method": seg.get("method", ""),
                    "length": seg.get("length", 0.0),
                    "start_point": _pnt_to_list(seg.get("start_point")),
                    "end_point":   _pnt_to_list(seg.get("end_point")),
                    "is_closed":   seg.get("is_closed", False),
                }
                for i, seg in enumerate(self.collected_segments)
            ],
            "total_length": sum(s["length"] for s in self.collected_segments),
        }

    def save_path_json(self):
        if not self.collected_segments:
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
            print(f"[IO] Saved {len(self.collected_segments)} segment(s) to {path}")
            QMessageBox.information(
                self, "Saved",
                f"Saved {len(self.collected_segments)} segment(s) to:\n{path}"
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
        n = len(data.get("segments", []))
        total = data.get("total_length", 0.0)
        self.loaded_metadata = data
        msg = (
            f"Loaded {n} segment(s) from:\n{path}\n\n"
            f"STEP file: {data.get('step_file', '(unknown)')}\n"
            f"Total length: {total:.2f} mm\n\n"
            f"Loaded path metadata. Reopen the STEP file to recreate geometry."
        )
        print(f"[IO] Loaded metadata: {n} segment(s), total {total:.2f} mm")
        QMessageBox.information(self, "Loaded", msg)

    def export_path_for_trajectory(self):
        if not self.collected_segments:
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
                f"Exported {len(self.collected_segments)} segment(s) to:\n{path}\n\n"
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

        if self.collected_segments and self.face_a is None and self.face_b is None:
            self.lbl_status.setText(
                "Segment added.\n"
                "Select next face pair to add another segment,\n"
                "or click 'Finish Path' to complete."
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
                f"Face A (Body {self.face_a[1]}) selected.\n"
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
                f"Face A:  Body {b_idx}  ✓\n"
                f"  Area: {area:.1f} mm²\n"
                f"  Center: ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f})"
            )
        else:
            self.lbl_face_a.setText("Face A:  —")
        if self.face_b is not None:
            _, b_idx, _, area, c = self.face_b
            self.lbl_face_b.setText(
                f"Face B:  Body {b_idx}  ✓\n"
                f"  Area: {area:.1f} mm²\n"
                f"  Center: ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f})"
            )
        else:
            self.lbl_face_b.setText("Face B:  —")

    def _update_segments_label(self):
        n = len(self.collected_segments)
        total = sum(s["length"] for s in self.collected_segments)
        self.lbl_segments.setText(f"Segments: {n}  |  Total: {total:.1f} mm")
        self.btn_finish.setEnabled(n > 0)
        self.btn_clear_segments.setEnabled(n > 0)
        self.btn_undo.setEnabled(n > 0)

    def _update_status_bar(self):
        if not self.solids:
            self.statusBar().showMessage("Ready — open a STEP file to begin")
            return
        fname = self.current_step_file.rsplit("/", 1)[-1].rsplit("\\", 1)[-1]
        n_seg = len(self.collected_segments)
        total = sum(s["length"] for s in self.collected_segments)
        self.statusBar().showMessage(
            f"File: {fname}  |  Bodies: {len(self.solids)}  |  "
            f"Segments: {n_seg}  |  Total: {total:.1f} mm"
        )


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    window.init_viewer()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
