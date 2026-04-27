from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Extend.TopologyUtils import TopologyExplorer

# STEP dosyasının yolu — küçük 2-body dosyamız
step_file = r"C:\Users\ASUS\Desktop\welding_project\3D-Step-Models-Library-master\CEL\TOBY\TOBY\TOBY.stp"

# 1) Dosyayı oku
reader = STEPControl_Reader()
status = reader.ReadFile(step_file)

if status != IFSelect_RetDone:
    print("HATA: Dosya okunamadı.")
    exit()

reader.TransferRoots()
shape = reader.OneShape()

# 2) Topology explorer oluştur — tüm shape için
top_explorer = TopologyExplorer(shape)

# 3) Tüm solid'leri (body) bir listeye al
solids = list(top_explorer.solids())

print(f"Toplam {len(solids)} adet body bulundu.\n")

# 4) Her body için ayrı bir TopologyExplorer açıp face/edge say
for i, solid in enumerate(solids, start=1):
    # Bu solid'i kendi başına gezecek bir explorer
    solid_explorer = TopologyExplorer(solid)
    
    face_count = solid_explorer.number_of_faces()
    edge_count = solid_explorer.number_of_edges()
    
    print(f"Body {i}: {face_count} face, {edge_count} edge")