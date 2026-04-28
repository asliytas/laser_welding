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
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE, TopAbs_SOLID
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopTools import (
    TopTools_IndexedDataMapOfShapeListOfShape,
    TopTools_HSequenceOfShape,
)
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBounds
from OCC.Core.BRep import BRep_Tool


PROXIMITY_THRESHOLD_MM = 5.0


class WeldingViewer(qtViewer3d):
    """3D viewer that forwards picked shapes to a callback."""

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

    COLOR_HIGHLIGHT = Quantity_Color(1.0, 0.85, 0.0, Quantity_TOC_RGB)
    COLOR_RED       = Quantity_Color(1.0, 0.0,  0.0, Quantity_TOC_RGB)
    COLOR_ORANGE    = Quantity_Color(1.0, 0.5,  0.0, Quantity_TOC_RGB)
    COLOR_CYAN      = Quantity_Color(0.0, 0.8,  1.0, Quantity_TOC_RGB)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Laser Welding – Phase 2")
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

        self.solids = []
        self.ais_shapes = []
        self.body_colors = []
        self.selected_bodies = []
        self.manual_edges = []
        self.manual_ais_list = []
        self.weld_ais_list = []

        self._create_menu()

    # ─── UI ───────────────────────────────────────────────────────────

    def _create_menu(self):
        open_action = QAction("Open STEP...", self)
        open_action.triggered.connect(self.open_step_file)
        self.menuBar().addMenu("File").addAction(open_action)

    def _create_side_panel(self):
        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        panel.setMinimumWidth(240)
        v = QVBoxLayout()

        # Mode selection
        mode_box = QGroupBox("Detection Mode")
        ml = QVBoxLayout()
        self.radio_auto   = QRadioButton("Auto (compute path)")
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

        # Proximity result (shown after validation)
        self.lbl_proximity = QLabel("")
        self.lbl_proximity.setWordWrap(True)
        self.lbl_proximity.setStyleSheet(
            "padding: 4px; color: #004400; font-weight: bold;"
        )
        v.addWidget(self.lbl_proximity)

        # Auto-mode buttons
        self.btn_reset = QPushButton("Reset Selection")
        self.btn_reset.clicked.connect(self.reset_all)
        v.addWidget(self.btn_reset)

        self.btn_confirm = QPushButton("Confirm && Compute Path")
        self.btn_confirm.clicked.connect(self.confirm_auto)
        self.btn_confirm.setEnabled(False)
        v.addWidget(self.btn_confirm)

        # Manual-mode buttons (hidden by default)
        self.btn_clear_edges = QPushButton("Clear Edges")
        self.btn_clear_edges.clicked.connect(self._clear_manual_edges)
        self.btn_clear_edges.setVisible(False)
        v.addWidget(self.btn_clear_edges)

        self.btn_apply_manual = QPushButton("Apply Manual Path")
        self.btn_apply_manual.clicked.connect(self.apply_manual)
        self.btn_apply_manual.setEnabled(False)
        self.btn_apply_manual.setVisible(False)
        v.addWidget(self.btn_apply_manual)

        # Path result
        self.lbl_path = QLabel("")
        self.lbl_path.setWordWrap(True)
        self.lbl_path.setStyleSheet("padding: 4px; color: #222;")
        v.addWidget(self.lbl_path)

        v.addStretch()
        panel.setLayout(v)
        return panel

    # ─── Mode switching ────────────────────────────────────────────────

    def _on_mode_toggled(self, auto_checked):
        ctx = self.viewer._display.GetContext()
        if auto_checked:
            for ais in self.ais_shapes:
                ctx.Activate(ais, 0, False)
                ctx.Deactivate(ais, 2)
            self.btn_confirm.setVisible(True)
            self.btn_clear_edges.setVisible(False)
            self.btn_apply_manual.setVisible(False)
            self._update_status()
        else:
            # Edge selection: mode 2 active, whole-shape mode 0 off
            for ais in self.ais_shapes:
                ctx.Activate(ais, 2, False)
                ctx.Deactivate(ais, 0)
            self.btn_confirm.setVisible(False)
            self.btn_clear_edges.setVisible(True)
            self.btn_apply_manual.setVisible(True)
            self.lbl_status.setText(
                "Manual mode:\nClick edges on the model\nto define the weld path."
            )

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
        self.selected_bodies.clear()
        self.manual_edges.clear()
        self.manual_ais_list.clear()
        self.weld_ais_list.clear()
        self.lbl_proximity.setText("")
        self.lbl_path.setText("")

        palette = [
            Quantity_Color(0.55, 0.55, 0.55, Quantity_TOC_RGB),
            Quantity_Color(0.25, 0.45, 0.90, Quantity_TOC_RGB),
            Quantity_Color(0.90, 0.40, 0.20, Quantity_TOC_RGB),
            Quantity_Color(0.35, 0.75, 0.30, Quantity_TOC_RGB),
            Quantity_Color(0.75, 0.30, 0.75, Quantity_TOC_RGB),
        ]

        for idx, solid in enumerate(self.solids):
            color = palette[idx % len(palette)]
            result = self.viewer._display.DisplayShape(solid, color=color, update=False)
            ais = result[0] if isinstance(result, list) else result
            self.ais_shapes.append(ais)
            self.body_colors.append(color)

        # If already in manual mode when file loaded, activate edge picking
        if self.radio_manual.isChecked():
            ctx = self.viewer._display.GetContext()
            for ais in self.ais_shapes:
                ctx.Activate(ais, 2, False)
                ctx.Deactivate(ais, 0)

        self.viewer._display.FitAll()
        self._update_status()

    # ─── Picking dispatch ──────────────────────────────────────────────

    def _on_shape_picked(self, shape):
        if self.radio_auto.isChecked():
            self._pick_body(shape)
        else:
            self._pick_edge(shape)

    # ─── Body picking (Auto mode) ──────────────────────────────────────

    def _pick_body(self, shape):
        idx = self._find_body_index(shape)
        if idx is None:
            print(f"Picked shape (type={shape.ShapeType()}) not matched to any body")
            return

        if idx in self.selected_bodies:
            self._deselect_body(idx)
            self.selected_bodies.remove(idx)
        else:
            if len(self.selected_bodies) >= 2:
                oldest = self.selected_bodies.pop(0)
                self._deselect_body(oldest)
            self.selected_bodies.append(idx)
            self._select_body(idx)

        print(f"Body selection: {self.selected_bodies}")
        self._update_status()

    def _find_body_index(self, shape):
        """Return 1-based solid index for any picked sub-shape."""
        for i, solid in enumerate(self.solids):
            if solid.IsSame(shape):
                return i + 1
        # Face → walk parent solid's faces
        if shape.ShapeType() == TopAbs_FACE:
            for i, solid in enumerate(self.solids):
                exp = TopExp_Explorer(solid, TopAbs_FACE)
                while exp.More():
                    if exp.Current().IsSame(shape):
                        return i + 1
                    exp.Next()
        return None

    def _select_body(self, idx):
        self.viewer._display.GetContext().SetColor(
            self.ais_shapes[idx - 1], self.COLOR_HIGHLIGHT, True
        )

    def _deselect_body(self, idx):
        self.viewer._display.GetContext().SetColor(
            self.ais_shapes[idx - 1], self.body_colors[idx - 1], True
        )

    # ─── Edge picking (Manual mode) ────────────────────────────────────

    def _pick_edge(self, shape):
        if shape.ShapeType() != TopAbs_EDGE:
            return
        props = GProp_GProps()
        brepgprop.LinearProperties(shape, props)
        length = props.Mass()
        print(f"Manual edge: {length:.2f} mm")

        self.manual_edges.append(shape)
        ais = AIS_Shape(shape)
        ais.SetColor(self.COLOR_CYAN)
        ais.SetWidth(4.0)
        self.viewer._display.GetContext().Display(ais, True)
        self.manual_ais_list.append(ais)
        self.btn_apply_manual.setEnabled(True)
        self.lbl_status.setText(f"Edges collected: {len(self.manual_edges)}")

    # ─── Reset helpers ─────────────────────────────────────────────────

    def reset_all(self):
        for idx in list(self.selected_bodies):
            self._deselect_body(idx)
        self.selected_bodies.clear()
        self._clear_manual_edges()
        self._clear_weld_display()
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

    # ─── Proximity validation (Adım B) ────────────────────────────────

    def _validate_proximity(self, a_idx, b_idx):
        """
        Uses BRepExtrema_DistShapeShape to measure the real minimum distance.
        Returns (is_valid, min_dist_mm, message).
        Works for any geometry type: prism, cylinder, cone, sphere, etc.
        """
        solid_a = self.solids[a_idx - 1]
        solid_b = self.solids[b_idx - 1]

        dist_calc = BRepExtrema_DistShapeShape(solid_a, solid_b)
        dist_calc.Perform()
        if not dist_calc.IsDone():
            return False, None, "Distance computation failed."

        d = dist_calc.Value()
        print(f"Min distance Body {a_idx} ↔ Body {b_idx}: {d:.4f} mm")

        if d > PROXIMITY_THRESHOLD_MM:
            return False, d, (
                f"Bodies are {d:.2f} mm apart "
                f"(threshold: {PROXIMITY_THRESHOLD_MM} mm)."
            )

        label = "touching" if d < 1e-3 else f"{d:.3f} mm gap"
        return True, d, f"Body {a_idx} ↔ Body {b_idx}: {label}"

    # ─── Geometry helpers ──────────────────────────────────────────────

    def _compute_section(self, a_idx, b_idx):
        """BRepAlgoAPI_Section works for any geometry: prism, cylinder, cone, sphere."""
        section = BRepAlgoAPI_Section(
            self.solids[a_idx - 1], self.solids[b_idx - 1]
        )
        section.Build()
        if not section.IsDone():
            return None
        result = section.Shape()
        return None if result.IsNull() else result

    def _extract_edges(self, shape, min_len=0.1):
        edges, noise = [], 0
        exp = TopExp_Explorer(shape, TopAbs_EDGE)
        while exp.More():
            edge = exp.Current()
            props = GProp_GProps()
            brepgprop.LinearProperties(edge, props)
            length = props.Mass()
            if length >= min_len:
                edges.append((edge, length))
            else:
                noise += 1
            exp.Next()
        if noise:
            print(f"  Discarded {noise} noise edge(s) < {min_len} mm")
        edges.sort(key=lambda x: x[1], reverse=True)
        return edges

    def _build_wires(self, edges_info):
        """Connect disjoint edges into wires; classify each as closed/open/arc."""
        if not edges_info:
            return []
        seq = TopTools_HSequenceOfShape()
        for edge, _ in edges_info:
            seq.Append(edge)
        wire_seq = TopTools_HSequenceOfShape()
        ShapeAnalysis_FreeBounds.ConnectEdgesToWires(seq, 1e-3, False, wire_seq)

        wires = []
        for i in range(1, wire_seq.Length() + 1):
            wire = wire_seq.Value(i)
            props = GProp_GProps()
            brepgprop.LinearProperties(wire, props)
            length = props.Mass()
            is_closed = BRep_Tool.IsClosed(wire)

            n_edges = 0
            exp = TopExp_Explorer(wire, TopAbs_EDGE)
            while exp.More():
                n_edges += 1
                exp.Next()

            wires.append({
                "wire": wire,
                "length": length,
                "closed": is_closed,
                "n_edges": n_edges,
            })

        wires.sort(key=lambda w: w["length"], reverse=True)
        return wires

    def _path_type_label(self, wire_info):
        """Adım D: classify path as closed loop, open path, or arc."""
        if wire_info["closed"]:
            return "Closed loop"
        if wire_info["n_edges"] == 1:
            return "Arc / single segment"
        return "Open path"

    def _show_wires(self, wires):
        """Display wires: primary in red (thick), secondary in orange (thin)."""
        ctx = self.viewer._display.GetContext()
        for i, w in enumerate(wires):
            ais = AIS_Shape(w["wire"])
            ais.SetColor(self.COLOR_RED if i == 0 else self.COLOR_ORANGE)
            ais.SetWidth(5.0 if i == 0 else 3.0)
            ctx.Display(ais, i == 0)
            self.weld_ais_list.append(ais)
        if len(wires) > 1:
            self.viewer._display.Repaint()

    # ─── Auto confirm ──────────────────────────────────────────────────

    def confirm_auto(self):
        if len(self.selected_bodies) != 2:
            return

        a, b = self.selected_bodies
        is_valid, dist, msg = self._validate_proximity(a, b)
        if not is_valid:
            QMessageBox.warning(self, "Bodies Too Far Apart", msg)
            self.lbl_proximity.setText("")
            return

        contact = "Touching" if dist is not None and dist < 1e-3 else f"Gap: {dist:.3f} mm"
        self.lbl_proximity.setText(contact)
        print(f"\n=== CONFIRMED ===  Body {a} ↔ Body {b}  ({contact})")

        self._clear_weld_display()
        intersection = self._compute_section(a, b)

        if intersection is None:
            QMessageBox.information(
                self, "No Intersection Curve",
                f"Bodies are within {PROXIMITY_THRESHOLD_MM} mm but share no "
                f"geometric intersection curve.\n\n"
                f"Switch to Manual mode to define the weld path by clicking edges."
            )
            return

        edges_info = self._extract_edges(intersection)
        print(f"Edges after filter: {len(edges_info)}")

        wires = self._build_wires(edges_info)
        print(f"Wires: {len(wires)}")
        for i, w in enumerate(wires, 1):
            print(f"  Wire {i}: {w['length']:.2f} mm  "
                  f"{'[closed]' if w['closed'] else '[open]'}  "
                  f"{w['n_edges']} edge(s)")

        if not wires:
            QMessageBox.warning(
                self, "No Weld Path",
                "Could not form a continuous path from the intersection edges."
            )
            return

        self._show_wires(wires)
        primary = wires[0]
        ptype = self._path_type_label(primary)
        self.lbl_path.setText(f"{ptype}\n{primary['length']:.1f} mm")

        detail = (
            f"{len(wires)} path(s) from {len(edges_info)} edge(s).\n\n"
            f"Primary path:\n"
            f"  Type:   {ptype}\n"
            f"  Length: {primary['length']:.2f} mm\n\n"
            f"RED = primary path"
        )
        if len(wires) > 1:
            detail += f"\nORANGE = {len(wires) - 1} secondary path(s)"
        QMessageBox.information(self, "Weld Path Found", detail)

    # ─── Manual apply ──────────────────────────────────────────────────

    def apply_manual(self):
        if not self.manual_edges:
            return

        edges_info = []
        for edge in self.manual_edges:
            props = GProp_GProps()
            brepgprop.LinearProperties(edge, props)
            edges_info.append((edge, props.Mass()))

        self._clear_weld_display()

        # Remove cyan preview highlights
        ctx = self.viewer._display.GetContext()
        for ais in self.manual_ais_list:
            ctx.Remove(ais, False)
        self.manual_ais_list.clear()

        wires = self._build_wires(edges_info)
        if not wires:
            QMessageBox.warning(
                self, "Path Error",
                "Could not build a wire from the selected edges.\n"
                "Try adding more connecting edges."
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

    # ─── Panel update ──────────────────────────────────────────────────

    def _update_status(self):
        n = len(self.selected_bodies)
        if not self.solids:
            self.lbl_status.setText("Open a STEP file to begin.")
        elif n == 0:
            self.lbl_status.setText("Click a body in the viewer to select it.")
        else:
            lines = [f"Selected ({n}/2):"]
            lines += [f"  • Body {i}" for i in self.selected_bodies]
            self.lbl_status.setText("\n".join(lines))
        self.btn_confirm.setEnabled(n == 2 and self.radio_auto.isChecked())


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    window.init_viewer()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
