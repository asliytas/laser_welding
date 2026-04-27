from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib

step_file = r"C:\Users\ASUS\Desktop\welding_project\3D-Step-Models-Library-master\CEL\TOBY\TOBY\TOBY.stp"

reader = STEPControl_Reader()
status = reader.ReadFile(step_file)

if status != IFSelect_RetDone:
    print("HATA: Dosya okunamadı.")
    exit()

reader.TransferRoots()
shape = reader.OneShape()

top_explorer = TopologyExplorer(shape)
solids = list(top_explorer.solids())

print(f"Toplam {len(solids)} adet body bulundu.\n")

for i, solid in enumerate(solids, start=1):
    solid_explorer = TopologyExplorer(solid)
    face_count = solid_explorer.number_of_faces()
    edge_count = solid_explorer.number_of_edges()
    
    # Bounding box hesapla
    bbox = Bnd_Box()
    brepbndlib.Add(solid, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    dx = xmax - xmin
    dy = ymax - ymin
    dz = zmax - zmin
    
    print(f"Body {i}: {face_count} face, {edge_count} edge")
    print(f"   Boyut: {dx:.2f} x {dy:.2f} x {dz:.2f} mm")
    print(f"   Konum: ({xmin:.2f}, {ymin:.2f}, {zmin:.2f})\n")