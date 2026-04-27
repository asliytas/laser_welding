from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_SOLID

# STEP dosyasının yolu
step_file = r"C:\Users\ASUS\Desktop\welding_project\3D-Step-Models-Library-master\CEL\TOBY\TOBY\TOBY.stp"

# STEP okuyucu nesnesi oluştur
reader = STEPControl_Reader()

# Dosyayı oku
status = reader.ReadFile(step_file)

if status != IFSelect_RetDone:
    print("HATA: Dosya okunamadı.")
    exit()

# Okunan içeriği OCC'nin iç yapısına aktar
reader.TransferRoots()

# Tüm shape'i tek bir compound olarak al
shape = reader.OneShape()

# İçindeki SOLID'leri (body'leri) sayalım
explorer = TopExp_Explorer(shape, TopAbs_SOLID)
solid_count = 0
while explorer.More():
    solid_count += 1
    explorer.Next()

print(f"Dosya: {step_file}")
print(f"{solid_count} adet body (solid) bulundu.") 