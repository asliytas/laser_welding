import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QAction
from OCC.Display.backend import load_backend
load_backend("pyqt5")

from OCC.Display.qtDisplay import qtViewer3d
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB


class MainWindow(QMainWindow):
    """3D viewer with STEP file loader."""
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Robotic Laser Welding - Phase 1")
        self.resize(1000, 700)
        
        # 3D viewer widget
        self.viewer = qtViewer3d(self)
        self.setCentralWidget(self.viewer)
        
        # Menu bar
        self._create_menu()
    
    def _create_menu(self):
        """Create the File menu with an Open action."""
        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("File")
        
        # "Open STEP..." action
        open_action = QAction("Open STEP...", self)
        open_action.triggered.connect(self.open_step_file)
        file_menu.addAction(open_action)
    
    def init_viewer(self):
        """Initialize OpenGL context. Must be called after show()."""
        self.viewer.InitDriver()
    
    def open_step_file(self):
        """Open a file dialog, load the STEP file, display its bodies."""
        # 1) Kullanicidan dosya yolu al
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Open STEP file",
            "",
            "STEP files (*.step *.stp *.STEP *.STP)"
        )
        
        # Kullanici "Cancel" derse bos string doner
        if not file_path:
            return
        
        # 2) STEP dosyasini oku
        reader = STEPControl_Reader()
        status = reader.ReadFile(file_path)
        
        if status != IFSelect_RetDone:
            print(f"ERROR: Could not read file: {file_path}")
            return
        
        reader.TransferRoots()
        shape = reader.OneShape()
        
        # 3) Body'leri ayikla
        top_explorer = TopologyExplorer(shape)
        solids = list(top_explorer.solids())
        print(f"Loaded {len(solids)} bodies from {file_path}")
        
        # 4) Onceki sahneyi temizle (yeni dosya acilirsa eskisi kalmasin)
        self.viewer._display.EraseAll()
        
        # 5) Renkleri tanimla — body 1 gri, body 2 mavi, sonrakiler farkli renklerde
        colors = [
            Quantity_Color(0.6, 0.6, 0.6, Quantity_TOC_RGB),  # gri
            Quantity_Color(0.2, 0.4, 0.9, Quantity_TOC_RGB),  # mavi
            Quantity_Color(0.9, 0.4, 0.2, Quantity_TOC_RGB),  # turuncu (3+ body olursa)
            Quantity_Color(0.4, 0.8, 0.3, Quantity_TOC_RGB),  # yesil
        ]
        
        # 6) Her body'yi ilgili renkte sahneye koy
        for i, solid in enumerate(solids):
            color = colors[i % len(colors)]  # renk listesi tukenirse bastan al
            self.viewer._display.DisplayShape(solid, color=color, update=False)
        
        # 7) Modeli ekrana sigdir
        self.viewer._display.FitAll()


def main():
    app = QApplication(sys.argv)
    
    window = MainWindow()
    window.show()
    window.init_viewer()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()