"""
Robotic Laser Welding — Phase 3   (main_updated_5.A.py)
========================================================
Mimari değişikliği: Body Pair Selection -> FACE Pair Selection.

Neden: body selection ile "iki gövde nerede temas ediyor" sorusunu
algoritmaya bırakmak ambiguous; touching/coincident yüzeylerde
Section + Proximity kombinasyonu bile güvenilir bir wire üretmiyor.

Yeni mantık:
  - Auto mode: kullanıcı her body'den birer face seçer (iki ayrı body'den).
    Sistem `BRepAlgoAPI_Section(face_a, face_b)` ile direkt
    iki yüzey arasındaki kesişim eğrisini hesaplar.
    Bu, kaynak yolunun semantic olarak DOĞRU tanımı.
  - Manual mode: aynı, kenar tıklayarak yol kurma.

Face-face Section neden body-body'den daha güvenilir:
  • 2 face zaten 2D → intersection 1D eğri (kaynak yolu)
  • Coplanar touching faces: overlap bölgesinin sınırı dönüyor
  • T-joint, L-joint, cylinder-on-plate gibi tüm temel kaynaklar net çıkıyor
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
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.Core.BRep import BRep_Tool


# ── Parametreler ────────────────────────────────────────────────────────────
PROXIMITY_THRESHOLD_MM = 5.0   # iki face arası max kaynak mesafesi
EDGE_MIN_LENGTH        = 0.01  # gürültü filtresi
WIRE_TOLERANCE         = 0.5   # ConnectEdgesToWires
CONTACT_TOL            = 0.5   # face boundary proximity fallback


# ── OCC selection mode sabitleri (TopAbs_ShapeEnum) ──────────────────────────
SEL_MODE_SHAPE = 0
SEL_MODE_EDGE  = 2
SEL_MODE_FACE  = 4


class WeldingViewer(qtViewer3d):
    """Picked shape'i parent'a ileten OCC viewer."""

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

    COLOR_FACE_A   = Quantity_Color(1.00, 0.85, 0.00, Quantity_TOC_RGB)  # sarı
    COLOR_FACE_B   = Quantity_Color(0.10, 0.70, 1.00, Quantity_TOC_RGB)  # mavi
    COLOR_RED      = Quantity_Color(1.00, 0.00, 0.00, Quantity_TOC_RGB)
    COLOR_ORANGE   = Quantity_Color(1.00, 0.50, 0.00, Quantity_TOC_RGB)
    COLOR_CYAN     = Quantity_Color(0.00, 0.85, 1.00, Quantity_TOC_RGB)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Laser Welding – Phase 3 (Face Pair)")
        self.resize(1300, 750)

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

        # Face pair state: each is None or (face, body_idx, ais_highlight)
        self.face_a = None
        self.face_b = None

        # Manual edge state
        self.manual_edges = []
        self.manual_ais_list = []

        # Weld result display
        self.weld_ais_list = []

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
        panel.setMinimumWidth(260)
        v = QVBoxLayout()

        # Mod seçimi
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

        # Status / talimat
        self.lbl_status = QLabel("Open a STEP file to begin.")
        self.lbl_status.setWordWrap(True)
        self.lbl_status.setStyleSheet(
            "padding: 8px; background: #f0f0f0; color: #333;"
        )
        v.addWidget(self.lbl_status)

        # Face A / Face B durumu (renkli)
        self.lbl_face_a = QLabel("Face A: —")
        self.lbl_face_a.setStyleSheet(
            "padding: 4px; color: #b08000; font-weight: bold;"
        )
        v.addWidget(self.lbl_face_a)

        self.lbl_face_b = QLabel("Face B: —")
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

        # Butonlar
        self.btn_reset = QPushButton("Reset Selection")
        self.btn_reset.clicked.connect(self.reset_all)
        v.addWidget(self.btn_reset)

        self.btn_confirm = QPushButton("Confirm && Compute Path")
        self.btn_confirm.clicked.connect(self.confirm_auto)
        self.btn_confirm.setEnabled(False)
        v.addWidget(self.btn_confirm)

        self.btn_clear_edges = QPushButton("Clear Edges")
        self.btn_clear_edges.clicked.connect(self._clear_manual_edges)
        self.btn_clear_edges.setVisible(False)
        v.addWidget(self.btn_clear_edges)

        self.btn_apply_manual = QPushButton("Apply Manual Path")
        self.btn_apply_manual.clicked.connect(self.apply_manual)
        self.btn_apply_manual.setEnabled(False)
        self.btn_apply_manual.setVisible(False)
        v.addWidget(self.btn_apply_manual)

        # Path sonucu
        self.lbl_path = QLabel("")
        self.lbl_path.setWordWrap(True)
        self.lbl_path.setStyleSheet("padding: 4px; color: #222;")
        v.addWidget(self.lbl_path)

        v.addStretch()
        panel.setLayout(v)
        return panel

    # ─── Mode switching ────────────────────────────────────────────────

    def _activate_selection_mode(self, mode):
        """Tüm body'lerde tek bir selection modunu aktif et."""
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
            self.btn_confirm.setVisible(True)
            self.btn_clear_edges.setVisible(False)
            self.btn_apply_manual.setVisible(False)
            self.lbl_face_a.setVisible(True)
            self.lbl_face_b.setVisible(True)
        else:
            self._activate_selection_mode(SEL_MODE_EDGE)
            self.btn_confirm.setVisible(False)
            self.btn_clear_edges.setVisible(True)
            self.btn_apply_manual.setVisible(True)
            self.lbl_face_a.setVisible(False)
            self.lbl_face_b.setVisible(False)
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
        print(f"Loaded {len(self.solids)} solid(s)")

        self.viewer._display.EraseAll()
        self.ais_shapes.clear()
        self.body_colors.clear()
        self.face_a = None
        self.face_b = None
        self.manual_edges.clear()
        self.manual_ais_list.clear()
        self.weld_ais_list.clear()
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

        # Aktif mode'a göre selection'ı kur
        if self.radio_auto.isChecked():
            self._activate_selection_mode(SEL_MODE_FACE)
        else:
            self._activate_selection_mode(SEL_MODE_EDGE)

        self.viewer._display.FitAll()
        self._update_status()

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
            print("Face: parent body bulunamadı")
            return

        # Aynı face zaten seçili mi? (toggle off)
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

        # Slot doldurma
        if self.face_a is None:
            self._set_face_a(face, body_idx)
        elif self.face_b is None:
            if body_idx == self.face_a[1]:
                QMessageBox.information(
                    self, "Same Body",
                    f"Bu face Body {body_idx}'den. Face A da aynı body'den.\n"
                    f"Lütfen FARKLI bir body'den face seçin."
                )
                return
            self._set_face_b(face, body_idx)
        else:
            # İkisi de dolu -> en eski (A) düşer, B → A olur, yeni face → B
            self._remove_face_highlight(self.face_a[2])
            old_face_b, body_b, ais_b = self.face_b
            self.viewer._display.GetContext().SetColor(ais_b, self.COLOR_FACE_A, True)
            self.face_a = (old_face_b, body_b, ais_b)
            self.face_b = None
            if body_idx == self.face_a[1]:
                QMessageBox.information(
                    self, "Same Body",
                    f"Bu face Body {body_idx}'den. Face A da aynı body'den.\n"
                    f"Lütfen FARKLI bir body'den face seçin."
                )
                self._update_status()
                return
            self._set_face_b(face, body_idx)

        self._update_status()

    def _set_face_a(self, face, body_idx):
        ais = self._highlight_face(face, self.COLOR_FACE_A)
        self.face_a = (face, body_idx, ais)
        print(f"Face A set (Body {body_idx})")

    def _set_face_b(self, face, body_idx):
        ais = self._highlight_face(face, self.COLOR_FACE_B)
        self.face_b = (face, body_idx, ais)
        print(f"Face B set (Body {body_idx})")

    def _highlight_face(self, face, color):
        """Picked face üzerine renkli AIS overlay koy; selectable değil."""
        ais = AIS_Shape(face)
        ais.SetColor(color)
        ctx = self.viewer._display.GetContext()
        ctx.Display(ais, False)
        ctx.Deactivate(ais)         # overlay üzerine tıklanmasın
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
        props = GProp_GProps()
        brepgprop.LinearProperties(edge, props)
        length = props.Mass()
        print(f"Manual edge: {length:.2f} mm")
        self.manual_edges.append(edge)
        ais = AIS_Shape(edge)
        ais.SetColor(self.COLOR_CYAN)
        ais.SetWidth(4.0)
        self.viewer._display.GetContext().Display(ais, True)
        self.manual_ais_list.append(ais)
        self.btn_apply_manual.setEnabled(True)
        self.lbl_status.setText(f"Edges collected: {len(self.manual_edges)}")

    # ─── Reset ─────────────────────────────────────────────────────────

    def reset_all(self):
        ctx = self.viewer._display.GetContext()
        if self.face_a is not None:
            ctx.Remove(self.face_a[2], False)
            self.face_a = None
        if self.face_b is not None:
            ctx.Remove(self.face_b[2], False)
            self.face_b = None
        for ais in self.manual_ais_list:
            ctx.Remove(ais, False)
        self.manual_edges.clear()
        self.manual_ais_list.clear()
        for ais in self.weld_ais_list:
            ctx.Remove(ais, False)
        self.weld_ais_list.clear()
        self.viewer._display.Repaint()
        self.btn_apply_manual.setEnabled(False)
        self.lbl_proximity.setText("")
        self.lbl_path.setText("")
        self._update_status()

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

    def _clear_weld_display(self):
        ctx = self.viewer._display.GetContext()
        for ais in self.weld_ais_list:
            ctx.Remove(ais, False)
        self.viewer._display.Repaint()
        self.weld_ais_list.clear()

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
        label = "touching" if dist < 1e-3 else f"{dist:.3f} mm gap"
        return True, dist, f"Body {body_a} ↔ Body {body_b}: {label}"

    # ─── Geometry algoritmaları ────────────────────────────────────────

    def _face_face_section(self, face_a, face_b):
        """İki face arası BRepAlgoAPI_Section. Coplanar overlap için
        overlap'in sınırını, intersecting için kesişim eğrisini döner."""
        section = BRepAlgoAPI_Section(face_a, face_b)
        section.ComputePCurveOn1(False)
        section.Approximation(False)
        section.Build()
        if not section.IsDone():
            return None
        result = section.Shape()
        return None if result.IsNull() else result

    def _extract_edges(self, shape, min_length=EDGE_MIN_LENGTH):
        edges = []
        exp = TopExp_Explorer(shape, TopAbs_EDGE)
        while exp.More():
            edge = exp.Current()
            props = GProp_GProps()
            brepgprop.LinearProperties(edge, props)
            length = props.Mass()
            if length >= min_length:
                edges.append((edge, length))
            exp.Next()
        edges.sort(key=lambda x: x[1], reverse=True)
        return edges

    def _face_boundary_proximity(self, face_a, face_b, gap_mm):
        """Section başarısızsa: face_a sınır kenarlarından face_b'ye
        yakın olanları (ve simetrik) topla. Coincident yüzeyler için."""
        threshold = max(CONTACT_TOL, gap_mm * 2.0 + 0.2)
        threshold = min(threshold, PROXIMITY_THRESHOLD_MM)

        edges = []
        for face_main, face_other in [(face_a, face_b), (face_b, face_a)]:
            exp = TopExp_Explorer(face_main, TopAbs_EDGE)
            while exp.More():
                edge = exp.Current()
                d = BRepExtrema_DistShapeShape(edge, face_other)
                d.Perform()
                if d.IsDone() and d.Value() <= threshold:
                    props = GProp_GProps()
                    brepgprop.LinearProperties(edge, props)
                    length = props.Mass()
                    if length >= EDGE_MIN_LENGTH:
                        edges.append((edge, length))
                exp.Next()
        return edges

    def _build_wires(self, edges_info):
        if not edges_info:
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
            props = GProp_GProps()
            brepgprop.LinearProperties(wire, props)
            length = props.Mass()
            if length < EDGE_MIN_LENGTH:
                continue
            is_closed = BRep_Tool.IsClosed(wire)
            n_edges = 0
            ex = TopExp_Explorer(wire, TopAbs_EDGE)
            while ex.More():
                n_edges += 1
                ex.Next()
            wires.append({
                "wire": wire, "length": length,
                "closed": is_closed, "n_edges": n_edges,
            })
        wires.sort(key=lambda w: w["length"], reverse=True)
        return wires

    def _path_type_label(self, w):
        if w["closed"]:
            return "Closed loop"
        if w["n_edges"] == 1:
            return "Arc / single segment"
        return "Open path"

    def _show_wires(self, wires):
        ctx = self.viewer._display.GetContext()
        for i, w in enumerate(wires):
            ais = AIS_Shape(w["wire"])
            ais.SetColor(self.COLOR_RED if i == 0 else self.COLOR_ORANGE)
            ais.SetWidth(5.0 if i == 0 else 3.0)
            ctx.Display(ais, i == 0)
            self.weld_ais_list.append(ais)
        if len(wires) > 1:
            self.viewer._display.Repaint()

    # ─── Confirm (Auto Face Pair) ──────────────────────────────────────

    def confirm_auto(self):
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
        print(f"\n=== CONFIRMED ===  Body {body_a} (Face) ↔ Body {body_b} (Face)  [{contact_str}]")

        self._clear_weld_display()

        # 1) Face-face Section
        sec = self._face_face_section(face_a, face_b)
        edges_info = []
        if sec is not None:
            edges_info = self._extract_edges(sec)
            print(f"  Face-Face Section: {len(edges_info)} edge(s)")

        # 2) Fallback: face boundary proximity
        if not edges_info:
            edges_info = self._face_boundary_proximity(face_a, face_b, gap or 0.0)
            print(f"  Face boundary proximity: {len(edges_info)} edge(s)")

        if not edges_info:
            QMessageBox.information(
                self, "No Weld Curve",
                "Could not detect a weld curve between the selected faces.\n\n"
                "Possible reasons:\n"
                "  • Faces meet only at a single point\n"
                "  • Faces are too small / numerically degenerate\n\n"
                "Switch to Manual mode to pick edges directly."
            )
            return

        wires = self._build_wires(edges_info)
        if not wires:
            QMessageBox.warning(
                self, "Wire Build Failed",
                f"Found {len(edges_info)} edge(s) but could not connect them.\n"
                f"Try Manual mode."
            )
            return

        print(f"Wires: {len(wires)}")
        for i, w in enumerate(wires, 1):
            print(f"  Wire {i}: {w['length']:.2f} mm  "
                  f"{'[closed]' if w['closed'] else '[open]'}  {w['n_edges']} edge(s)")

        self._show_wires(wires)
        primary = wires[0]
        ptype = self._path_type_label(primary)
        self.lbl_path.setText(f"{ptype}\n{primary['length']:.1f} mm")

        detail = (
            f"Primary path:\n"
            f"  Type:   {ptype}\n"
            f"  Length: {primary['length']:.2f} mm\n\n"
            f"RED = primary weld path"
        )
        if len(wires) > 1:
            detail += f"\nORANGE = {len(wires) - 1} secondary path(s)"
        QMessageBox.information(self, "Weld Path Found", detail)

    # ─── Apply (Manual) ────────────────────────────────────────────────

    def apply_manual(self):
        if not self.manual_edges:
            return
        edges_info = []
        for edge in self.manual_edges:
            props = GProp_GProps()
            brepgprop.LinearProperties(edge, props)
            edges_info.append((edge, props.Mass()))

        self._clear_weld_display()
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
        self._show_wires(wires)
        primary = wires[0]
        ptype = self._path_type_label(primary)
        self.lbl_path.setText(f"{ptype}\n{primary['length']:.1f} mm")
        print(f"Manual path: {ptype}, {primary['length']:.2f} mm")
        QMessageBox.information(
            self, "Manual Path Applied",
            f"Type:   {ptype}\nLength: {primary['length']:.2f} mm"
        )

    # ─── Status ────────────────────────────────────────────────────────

    def _update_status(self):
        if not self.solids:
            self.lbl_status.setText("Open a STEP file to begin.")
        elif self.radio_manual.isChecked():
            self.lbl_status.setText("Manual mode:\nClick edges on the model.")
        else:
            if self.face_a is None and self.face_b is None:
                self.lbl_status.setText(
                    "Auto (Face Pair):\n"
                    "1) Click a face on Body A.\n"
                    "2) Click a face on a DIFFERENT body."
                )
            elif self.face_a is not None and self.face_b is None:
                self.lbl_status.setText(
                    f"Face A (Body {self.face_a[1]}) selected.\n"
                    f"Now click a face on a different body."
                )
            else:
                self.lbl_status.setText("Both faces selected.\nClick Confirm.")

        if self.face_a is not None:
            self.lbl_face_a.setText(f"Face A:  Body {self.face_a[1]}  ✓")
        else:
            self.lbl_face_a.setText("Face A:  —")
        if self.face_b is not None:
            self.lbl_face_b.setText(f"Face B:  Body {self.face_b[1]}  ✓")
        else:
            self.lbl_face_b.setText("Face B:  —")

        self.btn_confirm.setEnabled(
            self.face_a is not None and self.face_b is not None
            and self.radio_auto.isChecked()
        )


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    window.init_viewer()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
