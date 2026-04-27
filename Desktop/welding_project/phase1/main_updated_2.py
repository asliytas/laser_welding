import sys
from turtle import shape
from OCC.Core.TopExp import TopExp_Explorer, topexp
from OCC.Core.TopTools import TopTools_IndexedDataMapOfShapeListOfShape

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QAction,
    QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QFrame,
    QMessageBox
)

from PyQt5.QtCore import Qt
from OCC.Display.backend import load_backend
from numpy import shape
load_backend("pyqt5")

from OCC.Display.qtDisplay import qtViewer3d
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCC.Core.AIS import AIS_Shape
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib



class WeldingViewer(qtViewer3d):
    """qtViewer3d subclass that handles face picking on left click."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        # Bu callback, MainWindow tarafindan set edilecek
        self.on_face_selected = None
    
    def mousePressEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
    
        # Sol tik = picking
        if event.button() == Qt.LeftButton:
            self._display.Select(x, y)
        
            context = self._display.GetContext()
        
            if context.NbSelected() > 0:
                context.InitSelected()
                while context.MoreSelected():
                    selected_shape = context.SelectedShape()
                    if self.on_face_selected is not None:
                        self.on_face_selected(selected_shape)
                    context.NextSelected()
            else:
                print("No shape under cursor")
    
        super().mousePressEvent(event)


class MainWindow(QMainWindow):
    """Main window with 3D viewer and STEP loader."""
    
    # Highlight rengi — secili body'ler bu renkte gosterilir
    
    
    HIGHLIGHT_COLOR = Quantity_Color(1.0, 0.85, 0.0, Quantity_TOC_RGB)  # parlak sari

    def _validate_selection(self):
        """
        Check if the two selected bodies can be welded.
        Returns (is_valid, message).
        """
        if len(self.selected_bodies) != 2:
            return False, "Need exactly 2 bodies selected."
        
        body_a_idx, body_b_idx = self.selected_bodies
        solid_a = self.solids[body_a_idx - 1]
        solid_b = self.solids[body_b_idx - 1]
        
        # Iki body'nin bounding box'larini hesapla
        bbox_a = Bnd_Box()
        brepbndlib.Add(solid_a, bbox_a)
        
        bbox_b = Bnd_Box()
        brepbndlib.Add(solid_b, bbox_b)
        
        # IsOut: True donerse kutular ayrik (kesismiyorlar)
        if bbox_a.IsOut(bbox_b):
            return False, (
                f"Body {body_a_idx} and Body {body_b_idx} are spatially separated.\n"
                f"Their bounding boxes do not overlap — no welding contact possible."
            )
        
        # Bbox boyutlarini bilgi amacli logla
        xa1, ya1, za1, xa2, ya2, za2 = bbox_a.Get()
        xb1, yb1, zb1, xb2, yb2, zb2 = bbox_b.Get()
        print(f"Body {body_a_idx} bbox: ({xa1:.1f}, {ya1:.1f}, {za1:.1f}) -> ({xa2:.1f}, {ya2:.1f}, {za2:.1f})")
        print(f"Body {body_b_idx} bbox: ({xb1:.1f}, {yb1:.1f}, {zb1:.1f}) -> ({xb2:.1f}, {yb2:.1f}, {zb2:.1f})")
        
        return True, f"Body {body_a_idx} and Body {body_b_idx} bounding boxes overlap — likely in contact."
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Robotic Laser Welding - Phase 2")
        self.resize(1200, 700)  # daha genis — panel sigsin
        
        # Custom viewer
        self.viewer = WeldingViewer(self)
        self.viewer.on_face_selected = self.handle_face_selection
        
        # Yan panel
        self.side_panel = self._create_side_panel()
        
        # Container: viewer (sol) + panel (sag) yan yana
        container = QWidget()
        layout = QHBoxLayout()
        layout.addWidget(self.viewer, stretch=4)       # 4 birim genislik
        layout.addWidget(self.side_panel, stretch=1)   # 1 birim genislik
        container.setLayout(layout)
        self.setCentralWidget(container)
        
        # State
        self.body_face_lists = []
        self.solids = []
        self.face_solid_map = None
        self.ais_shapes = []
        self.body_colors = []
        self.selected_bodies = []
        self.MAX_SELECTION = 2
        
        self._create_menu()
    
    def _create_menu(self):
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("File")
        
        open_action = QAction("Open STEP...", self)
        open_action.triggered.connect(self.open_step_file)
        file_menu.addAction(open_action)
    

    def _create_side_panel(self):
        """Create the right-side panel with selection info and buttons."""
        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        panel.setMinimumWidth(220)
        
        layout = QVBoxLayout()
        
        # Baslik
        title = QLabel("Selection")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)
        
        # Secim bilgileri (dinamik olarak guncellenecek)
        self.selection_label = QLabel("No bodies selected.")
        self.selection_label.setWordWrap(True)
        self.selection_label.setStyleSheet("padding: 8px; background-color: #f0f0f0; color: #333;")
        layout.addWidget(self.selection_label)
        
        # Reset butonu
        self.reset_btn = QPushButton("Reset Selection")
        self.reset_btn.clicked.connect(self.reset_selection)
        layout.addWidget(self.reset_btn)
        
        # Confirm butonu
        self.confirm_btn = QPushButton("Confirm Selection")
        self.confirm_btn.clicked.connect(self.confirm_selection)
        self.confirm_btn.setEnabled(False)  # bastlangicta pasif
        layout.addWidget(self.confirm_btn)
        
        # Bos alan — butonlari yukari it
        layout.addStretch()
        
        panel.setLayout(layout)
        return panel


    def init_viewer(self):
        self.viewer.InitDriver()
    
    def open_step_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open STEP file",
            "",
            "STEP files (*.step *.stp *.STEP *.STP)"
        )
    
        if not file_path:
            return
    
        reader = STEPControl_Reader()
        status = reader.ReadFile(file_path)
    
        if status != IFSelect_RetDone:
            print(f"ERROR: Could not read file: {file_path}")
            return
    
        reader.TransferRoots()
        shape = reader.OneShape()
    
        top_explorer = TopologyExplorer(shape)
        solids = list(top_explorer.solids())
        print(f"Loaded {len(solids)} bodies from {file_path}")
    
        self.viewer._display.EraseAll()
        self.solids = solids
        self.body_face_lists = []
        self.ais_shapes = []
        self.body_colors = []
        self.selected_bodies = []

        # Face → ait oldugu solid mapi olustur
        self.face_solid_map = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp.MapShapesAndAncestors(shape, TopAbs_FACE, 4, self.face_solid_map)

        colors = [
            Quantity_Color(0.6, 0.6, 0.6, Quantity_TOC_RGB),
            Quantity_Color(0.2, 0.4, 0.9, Quantity_TOC_RGB),
            Quantity_Color(0.9, 0.4, 0.2, Quantity_TOC_RGB),
            Quantity_Color(0.4, 0.8, 0.3, Quantity_TOC_RGB),
        ]

        for body_idx, solid in enumerate(solids, start=1):
            color = colors[(body_idx - 1) % len(colors)]
    
            # DisplayShape bir liste donduruyor — ilk eleman bizim AIS_Shape'imiz
            ais_list = self.viewer._display.DisplayShape(solid, color=color, update=False)
    
            # Donus tipi versiyona gore degisiyor — liste mi tek nesne mi?
            if isinstance(ais_list, list):
                ais_shape = ais_list[0]
            else:
                ais_shape = ais_list
    
            self.ais_shapes.append(ais_shape)
            self.body_colors.append(color)
    
            # Bu body'nin face'lerini sakla
            face_list = list(TopologyExplorer(solid).faces())
            self.body_face_lists.append(face_list)

        self.viewer._display.FitAll()
        self._update_panel()
    
    def handle_face_selection(self, picked_shape):


        """Called when a body is picked. Manages FIFO selection state."""
        body_idx = None
        for i, solid in enumerate(self.solids, start=1):
            if solid.IsSame(picked_shape):
                body_idx = i
                break
        
        if body_idx is None:
            print("Picked shape doesn't match any known body")
            return
        
        if body_idx in self.selected_bodies:
            self._deselect_body(body_idx)
            self.selected_bodies.remove(body_idx)
            print(f"Deselected: Body {body_idx}  |  selection: {self.selected_bodies}")
        else:
            if len(self.selected_bodies) >= self.MAX_SELECTION:
                oldest_idx = self.selected_bodies.pop(0)
                self._deselect_body(oldest_idx)
                print(f"  (queue full — dropped oldest: Body {oldest_idx})")
            
            self.selected_bodies.append(body_idx)
            self._select_body(body_idx)
            print(f"Selected: Body {body_idx}  |  selection: {self.selected_bodies}")
        
        self._update_panel()

    def reset_selection(self):
            """Clear all selections and restore original colors."""
            for body_idx in list(self.selected_bodies):
                self._deselect_body(body_idx)
            self.selected_bodies = []
            self._update_panel()
            print("Selection reset.")

    def confirm_selection(self):
        """User confirmed the two-body selection. Run validation, then trigger Phase 3."""
        if len(self.selected_bodies) != self.MAX_SELECTION:
            return
        
        is_valid, message = self._validate_selection()
        
        if not is_valid:
            # Uyari penceresi goster
            QMessageBox.warning(self, "Invalid Selection", message)
            print(f"VALIDATION FAILED: {message}")
            return
        
        # Validation OK — Faz 3'e gec
        body_a, body_b = self.selected_bodies
        print(f"\n=== CONFIRMED ===")
        print(f"Body {body_a} <-> Body {body_b}  selected for welding.")
        print(f"Validation: {message}")
        print(f"(Phase 3 will use these bodies to compute the weld path.)\n")
        
        # Kullaniciya da gosteren bir bilgi penceresi
        QMessageBox.information(
            self,
            "Selection Confirmed",
            f"Body {body_a} and Body {body_b} are confirmed for welding.\n\n"
            f"Phase 3 will compute the weld path."
        )


    def _update_panel(self):
        """Update the side panel UI based on current selection state."""
        if len(self.selected_bodies) == 0:
            self.selection_label.setText("No bodies selected.\n\nClick a body in the viewer to select it.")
            self.confirm_btn.setEnabled(False)
        else:
            lines = [f"Selected ({len(self.selected_bodies)}/{self.MAX_SELECTION}):"]
            for body_idx in self.selected_bodies:
                lines.append(f"  • Body {body_idx}")
            self.selection_label.setText("\n".join(lines))
            # Sadece tam 2 secili oldugunda Confirm aktif
            self.confirm_btn.setEnabled(len(self.selected_bodies) == self.MAX_SELECTION)               

    def _select_body(self, body_idx):
        """Apply highlight color to the body."""
        ais_shape = self.ais_shapes[body_idx - 1]
        context = self.viewer._display.GetContext()
        context.SetColor(ais_shape, self.HIGHLIGHT_COLOR, True)

    def _deselect_body(self, body_idx):
        """Restore the body's original color."""
        ais_shape = self.ais_shapes[body_idx - 1]
        original_color = self.body_colors[body_idx - 1]
        context = self.viewer._display.GetContext()
        context.SetColor(ais_shape, original_color, True)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    window.init_viewer()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()