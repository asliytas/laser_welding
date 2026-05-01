"""
Robotic Laser Welding — Phase 3   (main_updated_5.B.py)
========================================================
Güncellemeler (5.A → 5.B):

  Görev 1: Topological shared-edge detection (IsSame) — yeni ana algoritma
  Görev 2: Geometric coincident-edge fallback (endpoint + length + dist)
  Görev 3: _build_wires single-edge fast path (BRepBuilderAPI_MakeWire)
  Görev 4: confirm_auto: 4 katmanlı algoritma sırası
            (topological → geometric → section → error)
  Görev 5: Multi-segment path: Add Segment / Finish Path / Clear All
  Görev 6: UI status mesajları yeni akışa göre
  Görev 7: Bilgilendirici hata mesajları
  Görev 8: [ALGO] / [RESULT] tag'li debug log

Kullanıcı akışı:
  1) Auto mode'da iki face seç (ortak kenarı olan face'ler)
  2) "Add Segment" → segment biriktirilir, viewer'da kalıcı kalır
  3) Yeni face çifti seçip yine "Add Segment" → 2. segment
  4) "Finish Path" → toplam path özetlenir
"""

import sys

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QAction,
    QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QFrame,
    QMessageBox, QRadioButton, QGroupBox
)
from PyQt5.QtCore import Qt

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
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.Core.BRep import BRep_Tool


# ── Parametreler ────────────────────────────────────────────────────────────
PROXIMITY_THRESHOLD_MM   = 5.0    # iki face arası max kaynak mesafesi
EDGE_MIN_LENGTH          = 0.01   # gürültü filtresi
WIRE_TOLERANCE           = 0.5    # ConnectEdgesToWires
COINCIDENT_LEN_TOL       = 0.01   # iki edge'in uzunluk farkı toleransı
COINCIDENT_ENDPOINT_TOL  = 0.1    # endpoint eşleşme toleransı
COINCIDENT_DIST_TOL      = 0.1    # iki edge geometrik mesafe toleransı


# OCC selection mode sabitleri (TopAbs_ShapeEnum)
SEL_MODE_SHAPE = 0
SEL_MODE_EDGE  = 2
SEL_MODE_FACE  = 4


def _edge_endpoints(edge):
    """Edge'in start, mid, end noktalarını gp_Pnt olarak döner.
    Curve alınamazsa None döner."""
    try:
        result = BRep_Tool.Curve(edge)
        # pythonocc bazen (curve, u_min, u_max), bazen farklı tuple döner
        if result is None or len(result) < 3:
            return None
        curve, u_min, u_max = result[0], result[-2], result[-1]
        if curve is None:
            return None
        p_start = curve.Value(u_min)
        p_mid   = curve.Value((u_min + u_max) / 2.0)
        p_end   = curve.Value(u_max)
        return p_start, p_mid, p_end
    except Exception:
        return None


def _edge_length(edge):
    props = GProp_GProps()
    brepgprop.LinearProperties(edge, props)
    return props.Mass()


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


class MainWindow(QMainWindow):

    COLOR_FACE_A   = Quantity_Color(1.00, 0.85, 0.00, Quantity_TOC_RGB)
    COLOR_FACE_B   = Quantity_Color(0.10, 0.70, 1.00, Quantity_TOC_RGB)
    COLOR_CYAN     = Quantity_Color(0.00, 0.85, 1.00, Quantity_TOC_RGB)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Laser Welding – Phase 3 (Multi-Segment)")
        self.resize(1320, 780)

        self.viewer = WeldingViewer(self)
        self.viewer.on_shape_picked = self._on_shape_picked
        self.side_panel = self._create_side_panel()

        container = QWidget()
        layout = QHBoxLayout()
        layout.addWidget(self.viewer, stretch=4)
        layout.addWidget(self.side_panel, stretch=1)
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Geometri
        self.solids = []
        self.ais_shapes = []
        self.body_colors = []

        # Face pair state
        self.face_a = None  # (face, body_idx, ais_highlight)
        self.face_b = None

        # Manual edge state
        self.manual_edges = []
        self.manual_ais_list = []

        # Multi-segment state (Görev 5)
        # Her segment: { "wires": [...], "ais_list": [...], "method": str, "length": float }
        self.collected_segments = []

        self._create_menu()

    # ─── Menu ──────────────────────────────────────────────────────────

    def _create_menu(self):
        open_action = QAction("Open STEP...", self)
        open_action.triggered.connect(self.open_step_file)
        self.menuBar().addMenu("File").addAction(open_action)

    # ─── Side panel ────────────────────────────────────────────────────

    def _create_side_panel(self):
        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        panel.setMinimumWidth(280)
        v = QVBoxLayout()

        # Mod
        mode_box = QGroupBox("Detection Mode")
        ml = QVBoxLayout()
        self.radio_auto   = QRadioButton("Auto (pick 2 faces)")
        self.radio_manual = QRadioButton("Manual (pick edges)")
        self.radio_auto.setChecked(True)
        self.radio_auto.toggled.connect(self._on_mode_toggled)
        ml.addWidget(self.radio_auto)
        ml.addWidget(self.radio_manual)
        mode_box.setLayout(ml)
        v.addWidget(mode_box)

        # Status
        self.lbl_status = QLabel("Open a STEP file to begin.")
        self.lbl_status.setWordWrap(True)
        self.lbl_status.setStyleSheet(
            "padding: 8px; background: #f0f0f0; color: #333;"
        )
        v.addWidget(self.lbl_status)

        # Face A / B
        self.lbl_face_a = QLabel("Face A:  —")
        self.lbl_face_a.setStyleSheet(
            "padding: 4px; color: #b08000; font-weight: bold;"
        )
        v.addWidget(self.lbl_face_a)

        self.lbl_face_b = QLabel("Face B:  —")
        self.lbl_face_b.setStyleSheet(
            "padding: 4px; color: #006090; font-weight: bold;"
        )
        v.addWidget(self.lbl_face_b)

        # Yakınlık
        self.lbl_proximity = QLabel("")
        self.lbl_proximity.setWordWrap(True)
        self.lbl_proximity.setStyleSheet(
            "padding: 4px; color: #004400; font-weight: bold;"
        )
        v.addWidget(self.lbl_proximity)

        # Auto-mode butonları
        self.btn_reset_faces = QPushButton("Reset Faces")
        self.btn_reset_faces.clicked.connect(self.reset_face_selection)
        v.addWidget(self.btn_reset_faces)

        self.btn_add_segment = QPushButton("Add Segment")
        self.btn_add_segment.clicked.connect(self.add_segment)
        self.btn_add_segment.setEnabled(False)
        v.addWidget(self.btn_add_segment)

        # Multi-segment yönetim
        self.lbl_segments = QLabel("Segments collected: 0  |  Total: 0.0 mm")
        self.lbl_segments.setStyleSheet(
            "padding: 6px; background: #fff8e1; color: #555;"
        )
        v.addWidget(self.lbl_segments)

        self.btn_finish = QPushButton("Finish Path")
        self.btn_finish.clicked.connect(self.finish_path)
        self.btn_finish.setEnabled(False)
        v.addWidget(self.btn_finish)

        self.btn_clear_segments = QPushButton("Clear All Segments")
        self.btn_clear_segments.clicked.connect(self.clear_all_segments)
        self.btn_clear_segments.setEnabled(False)
        v.addWidget(self.btn_clear_segments)

        # Manual butonları
        self.btn_clear_edges = QPushButton("Clear Edges")
        self.btn_clear_edges.clicked.connect(self._clear_manual_edges)
        self.btn_clear_edges.setVisible(False)
        v.addWidget(self.btn_clear_edges)

        self.btn_apply_manual = QPushButton("Apply Manual Path")
        self.btn_apply_manual.clicked.connect(self.apply_manual)
        self.btn_apply_manual.setEnabled(False)
        self.btn_apply_manual.setVisible(False)
        v.addWidget(self.btn_apply_manual)

        # Path özeti
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
            for w in (self.btn_add_segment, self.btn_finish,
                      self.btn_clear_segments, self.btn_reset_faces,
                      self.lbl_face_a, self.lbl_face_b, self.lbl_segments):
                w.setVisible(True)
            self.btn_clear_edges.setVisible(False)
            self.btn_apply_manual.setVisible(False)
        else:
            self._activate_selection_mode(SEL_MODE_EDGE)
            for w in (self.btn_add_segment, self.btn_finish,
                      self.btn_clear_segments, self.lbl_face_a,
                      self.lbl_face_b, self.lbl_segments):
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
        print(f"[FILE] Loaded {len(self.solids)} solid(s) from {path}")

        self.viewer._display.EraseAll()
        self.ais_shapes.clear()
        self.body_colors.clear()
        self.face_a = None
        self.face_b = None
        self.manual_edges.clear()
        self.manual_ais_list.clear()
        for seg in self.collected_segments:
            pass  # AIS'ler EraseAll ile zaten gitti
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
        self._update_status()
        self._update_segments_label()

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

        # Toggle off
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

        # Slot doldur
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
            # İkisi de dolu → A düş, B → A, yeni → B
            self._remove_face_highlight(self.face_a[2])
            old_b_face, old_b_body, old_b_ais = self.face_b
            self.viewer._display.GetContext().SetColor(old_b_ais, self.COLOR_FACE_A, True)
            self.face_a = (old_b_face, old_b_body, old_b_ais)
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
        ais = self._highlight_face(face, self.COLOR_FACE_A)
        self.face_a = (face, body_idx, ais)
        print(f"[PICK] Face A → Body {body_idx}")

    def _set_face_b(self, face, body_idx):
        ais = self._highlight_face(face, self.COLOR_FACE_B)
        self.face_b = (face, body_idx, ais)
        print(f"[PICK] Face B → Body {body_idx}")

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
        """Sadece face A/B seçimini temizle. Biriken segmentlere dokunma."""
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
        """Biriken tüm segment'leri temizle."""
        ctx = self.viewer._display.GetContext()
        for seg in self.collected_segments:
            for ais in seg["ais_list"]:
                ctx.Remove(ais, False)
        self.collected_segments.clear()
        self.viewer._display.Repaint()
        self.lbl_path.setText("")
        self._update_segments_label()
        self._update_status()
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
        face_a, body_a, _ = self.face_a
        face_b, body_b, _ = self.face_b
        d = BRepExtrema_DistShapeShape(face_a, face_b)
        d.Perform()
        if not d.IsDone():
            return False, None, "Distance computation failed."
        dist = d.Value()
        if dist > PROXIMITY_THRESHOLD_MM:
            return False, dist, (
                f"Faces are {dist:.2f} mm apart "
                f"(threshold: {PROXIMITY_THRESHOLD_MM} mm)."
            )
        return True, dist, ""

    # ─── ALGO 1: Topological shared edges (Görev 1) ───────────────────

    def _find_shared_edges(self, face_a, face_b):
        """STEP shape sharing: face_a ve face_b aynı TopoDS_Edge instance'ını
        paylaşıyorsa onları bul (IsSame = TShape pointer karşılaştırması)."""
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

    # ─── ALGO 2: Geometric coincident edges (Görev 2) ─────────────────

    def _find_coincident_edges(self, face_a, face_b):
        """Shape sharing yoksa: edge'leri uzunluk + endpoint + mesafe ile
        karşılaştırarak çakışıkları bul."""
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

                # 1) Uzunluk yakınlığı
                if abs(length_a - length_b) > COINCIDENT_LEN_TOL:
                    continue

                # 2) Endpoint eşleşmesi (iki yönde)
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

                # 3) Geometrik mesafe (curve seviyesinde örtüşüyor mu)
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
        """Son çare: face-face Section. İki face teğet olmasa bile
        kesişiyorlarsa eğri çıkarabilir."""
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

    # ─── Wire builder (Görev 3) ────────────────────────────────────────

    def _build_wires(self, edges_info):
        if not edges_info:
            return []

        # Tek edge fast path
        if len(edges_info) == 1:
            edge, length = edges_info[0]
            try:
                mk = BRepBuilderAPI_MakeWire(edge)
                if not mk.IsDone():
                    return []
                wire = mk.Wire()
                return [{
                    "wire": wire,
                    "length": length,
                    "closed": BRep_Tool.IsClosed(wire),
                    "n_edges": 1,
                }]
            except Exception as e:
                print(f"[ERROR] Single-edge wire build: {e}")
                return []

        # Çoklu edge — ConnectEdgesToWires
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
                "wire": wire,
                "length": length,
                "closed": BRep_Tool.IsClosed(wire),
                "n_edges": n_edges,
            })
        wires.sort(key=lambda w: w["length"], reverse=True)
        return wires

    def _path_type_label(self, w):
        if w["closed"]:
            return "Closed loop"
        if w["n_edges"] == 1:
            return "Arc / single segment"
        return "Open path"

    # ─── Segment renkleri (Görev 5: red→orange gradient) ──────────────

    def _segment_color(self, idx):
        # Index ilerledikçe yeşil bileşeni artar: kırmızı → turuncu → sarımsı
        g = min(0.85, idx * 0.18)
        return Quantity_Color(1.0, g, 0.0, Quantity_TOC_RGB)

    # ─── Add Segment (Görev 4 + 5) ────────────────────────────────────

    def add_segment(self):
        """Yeni: confirm yerine Add Segment.
        Algoritma sırası: topological → geometric → section → error."""
        if self.face_a is None or self.face_b is None:
            return

        is_valid, gap, msg = self._validate_face_proximity()
        if not is_valid:
            QMessageBox.warning(self, "Faces Too Far Apart", msg)
            self.lbl_proximity.setText("")
            return

        contact_str = (
            "Touching" if gap is not None and gap < 1e-3
            else f"Gap: {gap:.3f} mm" if gap is not None
            else ""
        )
        self.lbl_proximity.setText(contact_str)

        face_a, body_a, _ = self.face_a
        face_b, body_b, _ = self.face_b
        print(f"\n=== ADD SEGMENT ===  Body {body_a} (Face) ↔ Body {body_b} (Face)  [{contact_str}]")

        edges_info = []
        method_used = None

        # 1) Topological shared
        edges_info = self._find_shared_edges(face_a, face_b)
        print(f"[ALGO] Topological shared edge search: {len(edges_info)} edge(s) found")
        if edges_info:
            method_used = "Topological shared"

        # 2) Geometric coincident
        if not edges_info:
            edges_info = self._find_coincident_edges(face_a, face_b)
            print(f"[ALGO] Geometric coincident edge search: {len(edges_info)} edge(s) found")
            if edges_info:
                method_used = "Geometric coincident"

        # 3) Section fallback
        if not edges_info:
            edges_info = self._section_fallback(face_a, face_b)
            print(f"[ALGO] Section fallback: {len(edges_info)} edge(s) found")
            if edges_info:
                method_used = "Section"

        # 4) Hata
        if not edges_info:
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

        wires = self._build_wires(edges_info)
        if not wires:
            QMessageBox.warning(
                self, "Wire Build Failed",
                f"{len(edges_info)} edge bulundu ama wire kurulamadı.\n"
                f"Manual mode'a geçmeyi deneyin."
            )
            return

        total_length = sum(w["length"] for w in wires)
        primary = wires[0]
        ptype = self._path_type_label(primary)
        print(f"[RESULT] Method used: {method_used}, total length: {total_length:.2f} mm")
        for i, w in enumerate(wires, 1):
            print(f"  Wire {i}: {w['length']:.2f} mm  "
                  f"{'[closed]' if w['closed'] else '[open]'}  {w['n_edges']} edge(s)")

        # Görev 5: Segment'e ekle ve görüntüle
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
        })

        # Face seçimini temizle, bir sonraki segment için hazır ol
        if self.face_a is not None:
            ctx.Remove(self.face_a[2], False)
            self.face_a = None
        if self.face_b is not None:
            ctx.Remove(self.face_b[2], False)
            self.face_b = None
        self.viewer._display.Repaint()

        self.lbl_path.setText(
            f"Last segment added:\n"
            f"  Type:   {ptype}\n"
            f"  Length: {total_length:.2f} mm\n"
            f"  Method: {method_used}"
        )
        self._update_segments_label()
        self._update_status()

    # ─── Finish Path ───────────────────────────────────────────────────

    def finish_path(self):
        if not self.collected_segments:
            return
        n = len(self.collected_segments)
        total = sum(s["length"] for s in self.collected_segments)
        lines = [f"Path finalized: {n} segment(s)", f"Total length: {total:.2f} mm", ""]
        for i, seg in enumerate(self.collected_segments, 1):
            lines.append(f"  {i}. {seg['type']:<22} {seg['length']:>7.2f} mm  ({seg['method']})")
        msg = "\n".join(lines)
        print(f"\n=== FINISH PATH ===")
        print(msg)
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

        # Manual path'i de bir segment olarak ekle
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

        primary = wires[0]
        total = sum(w["length"] for w in wires)
        ptype = self._path_type_label(primary)
        self.collected_segments.append({
            "wires": wires,
            "ais_list": segment_ais,
            "method": "Manual",
            "length": total,
            "type": ptype,
        })
        self.manual_edges.clear()
        self.btn_apply_manual.setEnabled(False)

        self.lbl_path.setText(
            f"Manual segment:\n  Type:   {ptype}\n  Length: {total:.2f} mm"
        )
        print(f"[RESULT] Manual path: {ptype}, {total:.2f} mm")
        self._update_segments_label()

    # ─── Status (Görev 6) ──────────────────────────────────────────────

    def _update_status(self):
        if not self.solids:
            self.lbl_status.setText("Open a STEP file to begin.")
            self.btn_add_segment.setEnabled(False)
            return

        if self.radio_manual.isChecked():
            self.lbl_status.setText("Manual mode:\nClick edges on the model.")
            return

        # Auto mode
        if self.collected_segments and self.face_a is None and self.face_b is None:
            # Bir segment eklendi; devam edebilir
            self.lbl_status.setText(
                f"Segment added.\n"
                f"Select next face pair to add another segment,\n"
                f"or click 'Finish Path' to complete."
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
            self.lbl_status.setText("Both faces selected.\nClick 'Add Segment'.")

        # Face A/B labels
        if self.face_a is not None:
            self.lbl_face_a.setText(f"Face A:  Body {self.face_a[1]}  ✓")
        else:
            self.lbl_face_a.setText("Face A:  —")
        if self.face_b is not None:
            self.lbl_face_b.setText(f"Face B:  Body {self.face_b[1]}  ✓")
        else:
            self.lbl_face_b.setText("Face B:  —")

        self.btn_add_segment.setEnabled(
            self.face_a is not None and self.face_b is not None
            and self.radio_auto.isChecked()
        )

    def _update_segments_label(self):
        n = len(self.collected_segments)
        total = sum(s["length"] for s in self.collected_segments)
        self.lbl_segments.setText(
            f"Segments collected: {n}  |  Total: {total:.1f} mm"
        )
        self.btn_finish.setEnabled(n > 0)
        self.btn_clear_segments.setEnabled(n > 0)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    window.init_viewer()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
