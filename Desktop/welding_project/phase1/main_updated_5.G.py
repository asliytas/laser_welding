"""
Robotic Laser Welding — Phase 3   (main_updated_5.G.py)
========================================================
5.B → 5.F geçmişi için 5.F dosyasının docstring'ine bakınız.

5.F → 5.G eklenenler:
  • Bridging path duplicate filter: aynı face çifti tekrar seçildiğinde
    daha önce eklenen path'ler dialog'da gösterilmez
  • _apply_bridging_paths içinde defense-in-depth duplicate check
  • Tüm path'ler zaten eklenmişse "Already Added" info dialog'u
  • Tek path tespit edildiğinde dialog'da uyarı: "diğer taraftaki weld path
    muhtemelen farklı bir face üzerinde"
  • Skipped duplicates için sonunda özet mesaj

Algoritma sırası (5.G):
  1) Topological shared-edge (IsSame, TShape pointer)
  2) Geometric coincident-edge (length + endpoint + dist)
  3) Section fallback
  4) Path-based bridging proposal (duplicate filter ile)
  5) Hata: Manual mode öner
"""

import sys
import json

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QAction,
    QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QFrame,
    QMessageBox, QRadioButton, QGroupBox, QListWidget, QListWidgetItem,
    QShortcut, QCheckBox, QDialog
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

CLEARANCE_THRESHOLD_MM   = 5.0
BRIDGING_MIN_DIST_MM     = 0.05
BRIDGING_MAX_DIST_MM     = 5.0

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


# ── Bridging Paths Dialog (5.G: single-path warning) ────────────────────────

class BridgingPathsDialog(QDialog):
    """
    Path-based bridging seçim dialog'u.
    5.G: tek path bulunduğunda kullanıcıya uyarı ekle.
    """

    def __init__(self, paths, single_side_warning=False, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Clearance Fit — Bridging Paths")
        self.paths = paths
        self.resize(640, 420)

        layout = QVBoxLayout()

        info = QLabel(
            f"Bu iki face arasında <b>{len(paths)}</b> adet bağımsız kaynak yolu "
            f"(path) bulundu.<br>"
            f"Hangisi(leri)ni segment olarak eklemek istiyorsunuz?<br><br>"
            f"<i>Not: Her path birden fazla edge'i birleştirilmiş tek bir "
            f"wire'dır.</i>"
        )
        info.setWordWrap(True)
        info.setStyleSheet("padding: 8px;")
        layout.addWidget(info)

        # 5.G: tek path durumu için ek uyarı
        if single_side_warning:
            single_note = QLabel(
                "<b>⚠ Sadece 1 path tespit edildi.</b><br>"
                "Diğer body'nin seçili face'inin kenarları karşı face'e yakın değil. "
                "Diğer taraftaki kaynak yolu muhtemelen <b>farklı bir face</b> üzerinde — "
                "ihtiyaç varsa o face'i seçip ayrı bir 'Add Segment' yapın, "
                "veya Manual mode kullanın."
            )
            single_note.setWordWrap(True)
            single_note.setStyleSheet(
                "padding: 6px; background: #fff3cd; color: #664d00; "
                "border: 1px solid #ffe69c; border-radius: 3px;"
            )
            layout.addWidget(single_note)

        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet("QListWidget::item { padding: 8px; }")
        for p in paths:
            text = (
                f"Path {p['side']} — Body {p['body_idx']} side  |  "
                f"length: {p['length']:.1f} mm  |  "
                f"edges: {p['n_edges']}  |  "
                f"type: {p['type']}  |  "
                f"avg gap: {p['avg_gap']:.2f} mm"
            )
            item = QListWidgetItem(text)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.list_widget.addItem(item)
        layout.addWidget(self.list_widget)

        warn = QLabel(
            "⚠ Bridging path'leri otomatik tespit edilir.\n"
            "Hangi tarafın doğru kaynak yolu olduğunu görsel olarak doğrulayın."
        )
        warn.setStyleSheet(
            "color: #888; font-style: italic; font-size: 11px; padding: 4px;"
        )
        warn.setWordWrap(True)
        layout.addWidget(warn)

        btn_layout = QHBoxLayout()
        self.btn_add = QPushButton("Add Selected Path(s) as Segments")
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
    """5.G: bridging duplicate filter + single-path UX hint."""

    COLOR_FACE_A   = Quantity_Color(1.00, 0.85, 0.00, Quantity_TOC_RGB)
    COLOR_FACE_B   = Quantity_Color(0.10, 0.70, 1.00, Quantity_TOC_RGB)
    COLOR_CYAN     = Quantity_Color(0.00, 0.85, 1.00, Quantity_TOC_RGB)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Laser Welding – Phase 3 (Path-Based Bridging)")
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
        self.lbl_legend = QLabel("Colors: distinct per segment")
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

    # ─── ALGO 4: Path-based bridging ──────────────────────────────────

    def _find_bridging_paths(self, face_a, face_b):
        body_a_idx = self.face_a[1] if self.face_a is not None else 0
        body_b_idx = self.face_b[1] if self.face_b is not None else 0

        paths = []

        for side, face_main, face_other, body_idx in [
            ("A", face_a, face_b, body_a_idx),
            ("B", face_b, face_a, body_b_idx),
        ]:
            near_edges = []
            edge_dists = []

            exp = TopExp_Explorer(face_main, TopAbs_EDGE)
            while exp.More():
                edge = exp.Current()
                length = _edge_length(edge)
                if length < EDGE_MIN_LENGTH:
                    exp.Next()
                    continue
                d = BRepExtrema_DistShapeShape(edge, face_other)
                d.Perform()
                if not d.IsDone():
                    exp.Next()
                    continue
                dist = d.Value()
                if BRIDGING_MIN_DIST_MM <= dist <= BRIDGING_MAX_DIST_MM:
                    near_edges.append((edge, length))
                    edge_dists.append(dist)
                exp.Next()

            if not near_edges:
                continue

            wires = self._build_wires(near_edges)
            if not wires:
                continue

            primary = wires[0]
            wire = primary["wire"]
            length = primary["length"]
            n_edges = primary["n_edges"]
            is_closed = primary["closed"]
            ptype = self._path_type_label(primary)
            start_pt, end_pt, _ = _wire_endpoints(wire)
            avg_gap = sum(edge_dists) / len(edge_dists) if edge_dists else 0.0

            paths.append({
                "side":         side,
                "body_idx":     body_idx,
                "edges":        [e for e, _ in near_edges],
                "wire":         wire,
                "length":       length,
                "n_edges":      n_edges,
                "is_closed":    is_closed,
                "type":         ptype,
                "start_point":  start_pt,
                "end_point":    end_pt,
                "avg_gap":      avg_gap,
            })

        return paths

    # ─── 5.G: bridging path duplicate filter ──────────────────────────

    def _bridging_path_already_added(self, path):
        """Bu path daha önce segment olarak eklenmiş mi? Eklenmişse 1-based
        segment index'i, değilse None döner."""
        candidate = {
            "start_point": path["start_point"],
            "end_point":   path["end_point"],
        }
        return self._is_duplicate_segment(candidate)

    def _apply_bridging_paths(self, paths, selected_indices):
        """Operatörün seçtiği path'leri segment olarak ekle.
        5.G: defense-in-depth duplicate check (filter zaten çalıştı,
        ama edge case'lerde duplicate gelirse skip et)."""
        if not selected_indices:
            return

        ctx = self.viewer._display.GetContext()
        added = 0
        skipped = 0
        last_method = ""
        last_total = 0.0
        last_type = ""

        for idx in selected_indices:
            p = paths[idx]

            # 5.G: defense-in-depth duplicate check
            dup_idx = self._bridging_path_already_added(p)
            if dup_idx is not None:
                print(f"[ALGO] Bridging-{p['side']} skipped: "
                      f"duplicate of segment {dup_idx}")
                skipped += 1
                continue

            wire        = p["wire"]
            length      = p["length"]
            ptype       = p["type"]
            is_closed   = p["is_closed"]
            n_edges     = p["n_edges"]
            side        = p["side"]
            avg_gap     = p["avg_gap"]
            start_pt    = p["start_point"]
            end_pt      = p["end_point"]

            method_used = f"Bridging-{side} (gap {avg_gap:.2f} mm)"

            wires_for_seg = [{
                "wire": wire,
                "length": length,
                "closed": is_closed,
                "n_edges": n_edges,
            }]

            seg_idx = len(self.collected_segments)
            color = self._segment_color(seg_idx)
            ais = AIS_Shape(wire)
            ais.SetColor(color)
            ais.SetWidth(5.0)
            ctx.Display(ais, False)

            self.collected_segments.append({
                "wires":       wires_for_seg,
                "ais_list":    [ais],
                "method":      method_used,
                "length":      length,
                "type":        ptype,
                "start_point": start_pt,
                "end_point":   end_pt,
                "is_closed":   is_closed,
            })
            added += 1
            last_method = method_used
            last_total = length
            last_type = ptype

        self.viewer._display.Repaint()

        # Face seçimi sıfırla
        if self.face_a is not None:
            ctx.Remove(self.face_a[2], False)
            self.face_a = None
        if self.face_b is not None:
            ctx.Remove(self.face_b[2], False)
            self.face_b = None
        self.viewer._display.Repaint()

        if added > 0:
            self.lbl_path.setText(
                f"⚠ Bridging path(s): {added} segment(s) added — verify visually\n\n"
                f"  Last: {last_type}, {last_total:.2f} mm  ({last_method})"
            )
        elif skipped > 0:
            self.lbl_path.setText(
                f"All selected paths were already added.\n"
                f"Skipped: {skipped}"
            )

        self._rebuild_segment_list()
        self._update_segments_label()
        self._update_status()
        self._update_status_bar()

        sides_added = [paths[i]['side'] for i in selected_indices
                       if self._bridging_path_already_added(paths[i]) is None]
        # Note: yukarıdaki list comp artık reliable değil (segment eklendi),
        # daha basit mesaj:
        print(f"[ALGO] Bridging paths applied: added={added}, skipped={skipped}")

        if skipped > 0 and added == 0:
            QMessageBox.information(
                self, "All Already Added",
                f"Seçilen tüm path'ler zaten daha önce segment olarak eklenmişti.\n"
                f"Skipped: {skipped}\n\n"
                f"Aynı path'i tekrar eklemek için önce listeden silin."
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

    # ─── Segment colors ───────────────────────────────────────────────

    def _segment_color(self, idx):
        r, g, b = SEGMENT_PALETTE[idx % len(SEGMENT_PALETTE)]
        return Quantity_Color(r, g, b, Quantity_TOC_RGB)

    def _segment_color_qt(self, idx):
        r, g, b = SEGMENT_PALETTE[idx % len(SEGMENT_PALETTE)]
        return QColor(int(r * 255), int(g * 255), int(b * 255))

    def _reassign_segment_colors(self):
        ctx = self.viewer._display.GetContext()
        for i, seg in enumerate(self.collected_segments):
            color = self._segment_color(i)
            for ais in seg["ais_list"]:
                ctx.SetColor(ais, color, True)
        self.viewer._display.Repaint()

    # ─── Add Segment (5.G: bridging duplicate filter) ─────────────────

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

        # Path-based bridging
        if not edges_info:
            print(f"[ALGO] All 3 algorithms returned 0 edges; gap={gap}")

            if gap is not None and gap < CLEARANCE_THRESHOLD_MM:
                print("[ALGO] Trying path-based bridging...")
                all_paths = self._find_bridging_paths(face_a, face_b)
                print(f"[ALGO] Bridging paths found: {len(all_paths)}")
                for p in all_paths:
                    print(f"  Path {p['side']}: {p['n_edges']} edge(s), "
                          f"length {p['length']:.2f} mm, "
                          f"closed={p['is_closed']}, "
                          f"avg_gap={p['avg_gap']:.2f} mm")

                # 5.G: zaten eklenmiş path'leri filtrele
                new_paths = []
                already_added = []
                for p in all_paths:
                    dup = self._bridging_path_already_added(p)
                    if dup is None:
                        new_paths.append(p)
                    else:
                        already_added.append((p, dup))
                        print(f"[ALGO] Path {p['side']} already in segments "
                              f"as Segment {dup} — filtered out")

                if not all_paths:
                    info_msg = (
                        f"Bu iki face arasında paylaşılan bir kenar bulunamadı.\n\n"
                        f"Gap: {gap:.2f} mm — yüzeyler yakın ama temas etmiyor.\n"
                        f"Bu durum bir clearance fit (boşluklu geçirme) olabilir.\n\n"
                        f"Bu tip geometrilerde tipik çözüm:\n"
                        f"  • Manual mode'a geçin\n"
                        f"  • Levhadaki delik kenarındaki daireyi doğrudan tıklayın\n"
                        f"  • Bu, plug/fillet welding için tipik kaynak yoludur"
                    )
                    QMessageBox.information(self, "Clearance Fit?", info_msg)
                    return

                if not new_paths:
                    # Tüm path'ler zaten eklenmiş
                    sides_already = ", ".join(
                        f"Path {p['side']} (Segment {idx})"
                        for p, idx in already_added
                    )
                    QMessageBox.information(
                        self, "Already Added",
                        f"Bu face çiftinden tüm bridging path'leri zaten "
                        f"segment listesine eklenmiş:\n\n"
                        f"  {sides_already}\n\n"
                        f"Aynı path'i ikinci kez eklemek için önce listeden silin.\n"
                        f"Veya farklı face'ler seçin."
                    )
                    return

                # Tek path tespit edildiyse uyarı göster (Path A var Path B yok veya tersi)
                single_warning = (len(all_paths) == 1)

                dialog = BridgingPathsDialog(
                    new_paths,
                    single_side_warning=single_warning,
                    parent=self
                )
                if dialog.exec_() == QDialog.Accepted:
                    selected = dialog.selected_indices()
                    if selected:
                        self._apply_bridging_paths(new_paths, selected)
                    return
                else:
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

    # ─── Segment list widget ──────────────────────────────────────────

    def _rebuild_segment_list(self):
        self.segment_list.blockSignals(True)
        self.segment_list.clear()
        _, _, gap_list = self._continuity_summary()
        for i, seg in enumerate(self.collected_segments):
            text = f"Segment {i + 1} — {seg['length']:.1f} mm — {seg['method']}"
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
            "version": "5.G",
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
