import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QAction
from PyQt5.QtCore import Qt
from OCC.Display.backend import load_backend
load_backend("pyqt5")

from OCC.Display.qtDisplay import qtViewer3d
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB


class WeldingViewer(qtViewer3d):
    """qtViewer3d subclass that logs mouse clicks while keeping default behavior."""
    
    def mousePressEvent(self, event):
        # 1) Once kendi mantigimiz: tiklamayi logla
        x = event.pos().x()
        y = event.pos().y()
        
        if event.button() == Qt.LeftButton:
            print(f"Left click at ({x}, {y})")
        elif event.button() == Qt.RightButton:
            print(f"Right click at ({x}, {y})")
        elif event.button() == Qt.MiddleButton:
            print(f"Middle click at ({x}, {y})")
        
        # 2) Sonra parent'in metodunu cagir — boylece dondurme/zoom calismaya devam eder
        super().mousePressEvent(event)


class MainWindow(QMainWindow):
    """Main window with 3D viewer and STEP loader."""
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Robotic Laser Welding - Phase 2")
        self.resize(1000, 700)
        
        # Custom viewer (mouse logging icin)
        self.viewer = WeldingViewer(self)
        self.setCentralWidget(self.viewer)
        
        self._create_menu()
    
    def _create_menu(self):
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("File")
        
        open_action = QAction("Open STEP...", self)
        open_action.triggered.connect(self.open_step_file)
        file_menu.addAction(open_action)
    
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
        
        colors = [
            Quantity_Color(0.6, 0.6, 0.6, Quantity_TOC_RGB),
            Quantity_Color(0.2, 0.4, 0.9, Quantity_TOC_RGB),
            Quantity_Color(0.9, 0.4, 0.2, Quantity_TOC_RGB),
            Quantity_Color(0.4, 0.8, 0.3, Quantity_TOC_RGB),
        ]
        
        for i, solid in enumerate(solids):
            color = colors[i % len(colors)]
            self.viewer._display.DisplayShape(solid, color=color, update=False)
        
        self.viewer._display.FitAll()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    window.init_viewer()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()