"""
Robotic Laser Welding — Test Geometry Generator  (Adım E)
=========================================================
Farklı temas türleri için STEP dosyası üretir.
Çıktı: phase1/test_geometries/ klasörü

Kullanım: python generate_test_geometries.py
          → main_updated_4.B.py ile her dosyayı aç ve test et.

Kapsanan temas tipleri:
  1. Yüzey-yüzey tam temas        (flat face, d=0)
  2. Yüzey-yüzey kısmi penetrasyon (overlap)
  3. Yüzey-yüzey boşluklu         (gap, d<5mm)
  4. Silindir-düzlem teması        (closed circle)
  5. Silindir-delik teması         (coincident surfaces)
  6. Eksenel silindirler           (reducer)
  7. Yan yana silindirler          (tangent line)
  8. T-birleşim                   (open line)
  9. L-birleşim                   (open line)
 10. Koni-düzlem                  (closed circle)
 11. Küre-düzlem                  (nokta temas → başarısız olmalı)
 12. Küre-küre                    (nokta temas → başarısız olmalı)
 13. Flanş-flanş (halka)           (ring closed path)
 14. Torus-düzlem                  (closed circle)
 15. Eğimli butt-joint             (angled open path)
"""

import os
import math

from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeSphere,
    BRepPrimAPI_MakeCone,
    BRepPrimAPI_MakeTorus,
)
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCC.Core.gp import (
    gp_Pnt, gp_Ax1, gp_Ax2, gp_Dir, gp_Vec, gp_Trsf,
)
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.IFSelect import IFSelect_RetDone


# ─── Çıktı klasörü ────────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_geometries")


# ─── Yardımcı fonksiyonlar ────────────────────────────────────────────────────

def save_step(filename, *shapes):
    """Birden fazla solid'i tek STEP dosyasına yaz (her biri ayrı body)."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    filepath = os.path.join(OUTPUT_DIR, filename)
    writer = STEPControl_Writer()
    for shape in shapes:
        writer.Transfer(shape, STEPControl_AsIs)
    ok = writer.Write(filepath) == IFSelect_RetDone
    status = "OK  " if ok else "FAIL"
    print(f"  [{status}]  {filename}")
    return ok


def box(dx, dy, dz, x=0.0, y=0.0, z=0.0):
    """Köşesi (x,y,z)'de olan dikdörtgen prizma."""
    return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), dx, dy, dz).Shape()


def cylinder(r, h, x=0.0, y=0.0, z=0.0, dir_z=True):
    """Z yönünde (varsayılan) silindir; tabanı (x,y,z)'de."""
    direction = gp_Dir(0, 0, 1) if dir_z else gp_Dir(1, 0, 0)
    ax = gp_Ax2(gp_Pnt(x, y, z), direction)
    return BRepPrimAPI_MakeCylinder(ax, r, h).Shape()


def sphere(r, x=0.0, y=0.0, z=0.0):
    return BRepPrimAPI_MakeSphere(gp_Pnt(x, y, z), r).Shape()


def cone(r_base, r_top, h, x=0.0, y=0.0, z=0.0):
    ax = gp_Ax2(gp_Pnt(x, y, z), gp_Dir(0, 0, 1))
    return BRepPrimAPI_MakeCone(ax, r_base, r_top, h).Shape()


def torus(R, r, x=0.0, y=0.0, z=0.0):
    """Major yarıçap R, minor yarıçap r; merkez (x,y,z)."""
    ax = gp_Ax2(gp_Pnt(x, y, z), gp_Dir(0, 0, 1))
    return BRepPrimAPI_MakeTorus(ax, R, r).Shape()


def cut(shape_a, shape_b):
    return BRepAlgoAPI_Cut(shape_a, shape_b).Shape()


def translate(shape, dx, dy, dz):
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(dx, dy, dz))
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def rotate_z(shape, angle_deg, cx=0.0, cy=0.0, cz=0.0):
    """shape'i (cx,cy,cz) noktasından geçen Z ekseni etrafında döndür."""
    trsf = gp_Trsf()
    ax = gp_Ax1(gp_Pnt(cx, cy, cz), gp_Dir(0, 0, 1))
    trsf.SetRotation(ax, math.radians(angle_deg))
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


# ─── Test geometrileri ────────────────────────────────────────────────────────

def t01_box_box_flat_touching():
    """
    Yüzey-yüzey tam temas (d=0), aynı boyutlu kutular üst üste.
    Beklenen: kapalı dikdörtgen yol (50×30 perimetre = 160 mm)
    Zorluk: KOLAY — Section ve Proximity her ikisi de çalışmalı.
    """
    a = box(50, 30, 20)
    b = box(50, 30, 20, z=20)          # tam oturuyor
    save_step("01_box_box_flat_touching.step", a, b)


def t02_box_box_overlap_1mm():
    """
    1mm penetrasyon (overlap). Section algoritması için klasik durum.
    Beklenen: kapalı dikdörtgen yol z≈19'da
    Zorluk: KOLAY — Section ideal çalışır.
    """
    a = box(50, 30, 20)
    b = box(50, 30, 20, z=19)          # 1 mm içine giriyor
    save_step("02_box_box_overlap_1mm.step", a, b)


def t03_box_box_gap_3mm():
    """
    3mm boşluk (threshold=5mm içinde). Section boş döner.
    Beklenen: edge-proximity yaklaşımı temas yüzeylerinin kenarlarını bulur.
    Zorluk: ORTA — proximity algoritmasını test eder.
    """
    a = box(50, 30, 20)
    b = box(50, 30, 20, z=23)          # 3mm gap
    save_step("03_box_box_gap_3mm.step", a, b)


def t04_box_box_partial_shifted():
    """
    Kısmi örtüşme: ikinci kutu yanda kaymış, sadece yarısı üstte.
    Beklenen: açık veya kapalı L-şekilli yol.
    Zorluk: ORTA.
    """
    a = box(60, 40, 20)
    b = box(60, 40, 20, x=30, z=20)   # X'te 30mm kaymış, üstte duruyor
    save_step("04_box_box_partial_shifted.step", a, b)


def t05_t_joint():
    """
    T-birleşim: dikey plaka yatay plakanın üstünde duruyor (kenar teması).
    Beklenen: açık doğru yol (uzunluk = 80 mm).
    Zorluk: KOLAY — flat face, proximity ile kenar bulunur.
    """
    base     = box(80, 60, 10)
    vertical = box(80, 8, 40, y=26, z=10)   # yatay merkezde, tam oturuyor
    save_step("05_t_joint.step", base, vertical)


def t06_l_joint():
    """
    L-birleşim: iki plaka 90° açıyla, ortak kenar boyunca temas.
    Beklenen: açık doğru yol (uzunluk = 60 mm).
    Zorluk: KOLAY.
    """
    horizontal = box(80, 60, 8)
    vertical   = box(8, 60, 50, x=80, z=0)  # sağ yüzde tam temas
    save_step("06_l_joint.step", horizontal, vertical)


def t07_cylinder_on_plate():
    """
    Silindir düz plakaya oturuyor (boru-plaka kaynağı).
    Beklenen: kapalı daire yol (R=15, çevre ≈ 94.2 mm).
    Zorluk: ORTA — curved-flat temas, Section genellikle başarısız,
            proximity dairenin tabanını bulmalı.
    """
    plate = box(100, 100, 10)
    cyl   = cylinder(r=15, h=40, x=50, y=50, z=10)  # taban plaka üstünde
    save_step("07_cylinder_on_plate.step", plate, cyl)


def t08_cylinder_in_hole_clearance():
    """
    Pin-delik: silindir bir deliğe oturuyor, 0.2mm radyal boşluk var.
    Boşluk < CONTACT_EDGE_TOL (0.5mm), dolayısıyla proximity yakalamalı.
    Beklenen: kapalı daire yol (z=20'de, deliğin üst kenarı).
    Zorluk: ZOR — coincident-benzeri surfaces, Section boş döner.
    """
    # Delikli blok
    blok    = box(60, 60, 20)
    drill   = cylinder(r=12.0, h=22, x=30, y=30, z=-1)  # tam delik
    blok_delik = cut(blok, drill)

    # Pin: R=11.8 (0.2mm boşluk), uzunluğu bloktan 10mm taşıyor
    pin = cylinder(r=11.8, h=30, x=30, y=30, z=-10)     # pin taşkını z=-10..20
    save_step("08_cylinder_in_hole_clearance.step", blok_delik, pin)


def t09_coaxial_cylinders_reducer():
    """
    Aynı eksende farklı çaplı iki silindir (reducer/indirgeyici boru).
    Beklenen: kapalı daire yol (join noktasında, küçük silindir R=12).
    Zorluk: ORTA — flat face (annular ring), Section daire döndürmeli.
    """
    c_alt = cylinder(r=20, h=30, x=50, y=50, z=0)
    c_ust = cylinder(r=12, h=30, x=50, y=50, z=30)  # üstte, tam oturuyor
    save_step("09_coaxial_cylinders_reducer.step", c_alt, c_ust)


def t10_cylinder_cylinder_side_tangent():
    """
    Yan yana eşit silindirler, dış yüzeyden teğet temas (generator line).
    Beklenen: teorik olarak bir doğru (dejenere), yay yok.
            → Büyük ihtimalle proximity yaklaşımı da başarısız;
              kullanıcıyı manual moda yönlendirmeli.
    Zorluk: ÇOK ZOR — tangent curved surfaces.
    """
    c1 = cylinder(r=15, h=60, x=15, y=30, z=0)
    c2 = cylinder(r=15, h=60, x=45, y=30, z=0)  # dış yüzeyden teğet (x=30)
    save_step("10_cylinder_cylinder_side_tangent.step", c1, c2)


def t11_cone_on_plate():
    """
    Koni tabanıyla düz plakaya oturuyor.
    Beklenen: kapalı daire yol (R=20, çevre ≈ 125.7 mm).
    Zorluk: ORTA — curved-flat, proximity taban halkasını bulmalı.
    """
    plate = box(120, 120, 10)
    cn    = cone(r_base=20, r_top=0, h=40, x=60, y=60, z=10)
    save_step("11_cone_on_plate.step", plate, cn)


def t12_sphere_on_plate():
    """
    Küre düz plakaya teğet temas (nokta teması).
    Beklenen: BAŞARISIZ — hiçbir eğri yok.
            Kod "Point contact" mesajı vermeli ve Manual moda yönlendirmeli.
    Zorluk: GRAFİK TEST — graceful failure doğrulaması.
    """
    plate = box(100, 100, 10)
    sph   = sphere(r=25, x=50, y=50, z=35)  # merkez z=35 → taban z=10'da
    save_step("12_sphere_on_plate.step", plate, sph)


def t13_sphere_sphere_tangent():
    """
    İki eşit küre dışarıdan teğet temas.
    Beklenen: BAŞARISIZ — nokta teması, eğri yok.
    Zorluk: GRAFİK TEST.
    """
    s1 = sphere(r=20, x=20, y=50, z=50)
    s2 = sphere(r=20, x=60, y=50, z=50)  # tam teğet, merkez arası = 40 = r1+r2
    save_step("13_sphere_sphere_tangent.step", s1, s2)


def t14_flange_to_flange():
    """
    Dairesel flanş üstüne flanş (halka şeklinde temas).
    Flanş A: delikli disk; Flanş B: düz disk; üst üste oturuyor.
    Beklenen: kapalı halka yol (annular ring perimeter).
    Zorluk: ZOR — karmaşık yüzey, Section + proximity kombinasyonu.
    """
    # Flanş A: dış R=40, iç delik R=15, 4 cıvata deliği R=4
    disk_a = cylinder(r=40, h=8, x=0, y=0, z=0)
    disk_a = cut(disk_a, cylinder(r=15, h=10, x=0, y=0, z=-1))
    bolt_positions = [(28, 0), (0, 28), (-28, 0), (0, -28)]
    for bx, by in bolt_positions:
        disk_a = cut(disk_a, cylinder(r=4, h=10, x=bx, y=by, z=-1))

    # Flanş B: düz disk (deliksiz), üstünde oturuyor
    disk_b = cylinder(r=40, h=8, x=0, y=0, z=8)
    save_step("14_flange_to_flange.step", disk_a, disk_b)


def t15_torus_on_plate():
    """
    Torus (halka) düz plakaya oturuyor (alt yüzeyden teğet daire).
    Beklenen: kapalı daire yol (major R=25 etrafında küçük bir çember).
    Zorluk: ÇOK ZOR — curved-flat tangent contact.
    """
    plate = box(120, 120, 10)
    # Torus: major R=25, minor R=8; merkezi z=18'de → alt noktası z=18-8=10
    tr = torus(R=25, r=8, x=60, y=60, z=18)
    save_step("15_torus_on_plate.step", plate, tr)


def t16_angled_butt_joint():
    """
    45° eğimli alın kaynağı: iki kutu 45° açıyla birleşiyor.
    Beklenen: diyagonal açık path.
    Zorluk: ZOR — angled face contact.
    """
    # Kutu A: 50×30×20
    a = box(50, 30, 20)

    # Kutu B: 50×30×20, merkezi etrafında 45° döndürülmüş, A'nın yanına yerleştirilmiş
    b_raw = box(50, 30, 20, x=-25, y=-15)   # merkezlenmiş
    b_rot = rotate_z(b_raw, 45, cx=0, cy=0, cz=10)

    # Döndürülmüş B'yi A'nın sağ yanına taşı (yaklaşık temas)
    # 45°'de bbox genişliği = (50+30)/√2 ≈ 56.6 → A sağı x=50, B solu ≈ x=50
    b = translate(b_rot, 75, 15, 0)
    save_step("16_angled_butt_joint.step", a, b)


def t17_box_on_cylinder_saddle():
    """
    Kutu silindirin üstüne oturuyor (eyer kaynağı — saddle weld).
    Düz yüzey eğrisel yüzeyle temas: kesişim eğrisi bir elips/yay.
    Beklened: açık eğri yol (ark).
    Zorluk: ZOR — flat-curved, Section çalışmalı (overlap var).
    """
    # Silindir: yatay (X yönünde), R=20, h=100
    ax = gp_Ax2(gp_Pnt(0, 50, 30), gp_Dir(1, 0, 0))
    cyl = BRepPrimAPI_MakeCylinder(ax, 20, 100).Shape()

    # Kutu: silindirin üstüne 2mm giriyor (overlap → Section çalışır)
    blk = box(30, 30, 30, x=35, y=35, z=48)  # z=48 → z=50'ye kadar, sil z=50'de
    save_step("17_box_on_cylinder_saddle.step", cyl, blk)


def t18_two_plates_overlapping_cross():
    """
    Çapraz plakalar: biri +45°, diğeri -45° döndürülmüş, ortadan kesişiyor.
    Beklenen: X şeklinde kesişim eğrisi (iki çizgi / açık yollar).
    Zorluk: ORTA — Section çalışmalı (gerçek kesişim).
    """
    # Plaka A: 100×10×4, yatay, ortalanmış
    plate_a = box(100, 10, 4, x=-50, y=-5, z=0)

    # Plaka B: aynı boyut, 90° döndürülmüş
    plate_b_raw = box(100, 10, 4, x=-50, y=-5, z=0)
    plate_b = rotate_z(plate_b_raw, 90)

    # Her ikisi z=0..4 aralığında, birbiriyle kesişiyor
    save_step("18_two_plates_cross.step", plate_a, plate_b)


# ─── Ana program ──────────────────────────────────────────────────────────────

TESTS = [
    # (açıklama, fonksiyon)
    ("01  Box-Box flat touching              [closed rect,  KOLAY]", t01_box_box_flat_touching),
    ("02  Box-Box 1mm overlap                [closed rect,  KOLAY]", t02_box_box_overlap_1mm),
    ("03  Box-Box 3mm gap                    [closed rect,  ORTA] ", t03_box_box_gap_3mm),
    ("04  Box-Box partial shifted            [closed poly,  ORTA] ", t04_box_box_partial_shifted),
    ("05  T-joint                            [open line,    KOLAY]", t05_t_joint),
    ("06  L-joint                            [open line,    KOLAY]", t06_l_joint),
    ("07  Cylinder on plate                  [closed circle,ORTA] ", t07_cylinder_on_plate),
    ("08  Cylinder in hole (0.2mm gap)       [closed circle,ZOR]  ", t08_cylinder_in_hole_clearance),
    ("09  Coaxial cylinders reducer          [closed circle,ORTA] ", t09_coaxial_cylinders_reducer),
    ("10  Cylinders side tangent             [line/FAIL,    ÇOK ZOR]", t10_cylinder_cylinder_side_tangent),
    ("11  Cone on plate                      [closed circle,ORTA] ", t11_cone_on_plate),
    ("12  Sphere on plate                    [FAIL graceful,TEST] ", t12_sphere_on_plate),
    ("13  Sphere-sphere tangent              [FAIL graceful,TEST] ", t13_sphere_sphere_tangent),
    ("14  Flange-to-flange                   [ring closed,  ZOR]  ", t14_flange_to_flange),
    ("15  Torus on plate                     [circle,       ÇOK ZOR]", t15_torus_on_plate),
    ("16  Angled butt joint 45°              [open line,    ZOR]  ", t16_angled_butt_joint),
    ("17  Box on cylinder (saddle)           [arc,          ZOR]  ", t17_box_on_cylinder_saddle),
    ("18  Two plates cross                   [X-shape,      ORTA] ", t18_two_plates_overlapping_cross),
]


if __name__ == "__main__":
    print("=" * 65)
    print("  Welding Test Geometry Generator")
    print(f"  Çıktı: {OUTPUT_DIR}")
    print("=" * 65)
    print()

    passed, failed = 0, 0
    for desc, fn in TESTS:
        print(f"  {desc}")
        try:
            fn()
            passed += 1
        except Exception as e:
            print(f"         HATA: {e}")
            failed += 1
        print()

    print("=" * 65)
    print(f"  Sonuç: {passed} başarılı / {failed} hatalı / {len(TESTS)} toplam")
    print("=" * 65)
    print()
    print("  Test sirasi onerisi (main_updated_4.B.py ile):")
    print("  1. 01, 02 -> Section calismali (baseline)")
    print("  2. 07, 09 -> Proximity devreye girmeli")
    print("  3. 08     -> ZOR: coincident surface")
    print("  4. 12, 13 -> Graceful failure (nokta temas mesaji)")
    print("  5. 03     -> Gap detection")
    print("  6. 14, 17 -> Karmasik geometriler")
