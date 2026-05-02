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
    BRepPrimAPI_MakePrism,
)
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Transform,
)
from OCC.Core.GC import GC_MakeArcOfCircle
from OCC.Core.gp import (
    gp_Pnt, gp_Ax1, gp_Ax2, gp_Dir, gp_Vec, gp_Trsf,
)
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.IFSelect import IFSelect_RetDone


# ─── Çıktı klasörü ────────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_geometries")

SAVE_NAME_OVERRIDES = {
    "01_box_box_flat_touching.step": "OKEY/01_box_box_flat_touching.step",
    "03_box_box_gap_3mm.step": "OKEY/02.A_box_box_gap_3mm.step",
    "19_box_box_gap_0p5mm.step": "OKEY/02.B_box_box_gap_0p5mm.step",
    "20_box_box_gap_4p8mm.step": "OKEY/02.C_box_box_gap_4p8mm.step",
    "21_box_box_gap_6mm_too_far.step": "OKEY/02.D_box_box_gap_6mm_too_far.step",
    "27_three_boxes_stacked_two_gaps.step": "OKEY/2.E_three_boxes_stacked_two_gaps.step",
    "04_box_box_partial_shifted.step": "OKEY/03_box_box_partial_shifted.step",
    "05_t_joint.step": "OKEY/04_t_joint.step",
    "06_l_joint.step": "OKEY/05_l_joint.step",
    "07_cylinder_on_plate.step": "OKEY/06_cylinder_on_plate.step",
    "09_coaxial_cylinders_reducer.step": "OKEY/08_coaxial_cylinders_reducer.step",
    "10_cylinder_cylinder_side_tangent.step": "OKEY/09_cylinder_cylinder_side_tangent.step",
    "12_sphere_on_plate.step": "OKEY/10_sphere_on_plate.step",
    "13_sphere_sphere_tangent.step": "OKEY/11_sphere_sphere_tangent.step",
    "14_flange_to_flange.step": "OKEY/12_flange_to_flange.step",
    "15_torus_on_plate.step": "OKEY/13_torus_on_plate.step",
    "22_box_box_corner_touch_only.step": "OKEY/14_box_box_corner_touch_only.step",
    "23_box_box_edge_touch_only.step": "OKEY/15_box_box_edge_touch_only.step",
    "33_arc_rail_radial_touch.step": "OKEY/30_arc_rail_radial_touch.step",

    "25_box_box_partial_side_gap_3mm.step": "16_box_box_partial_side_gap_3mm.step",
    "26_box_box_offset_gap_3mm.step": "23_box_box_offset_gap_3mm.step",
    "29_cylinder_on_plate_gap_2mm.step": "26_cylinder_on_plate_gap_2mm.step",
    "31_t_joint_gap_2mm.step": "28_t_joint_gap_2mm.step",
    "34_arc_rail_radial_gap_2mm.step": "31_arc_rail_radial_gap_2mm.step",
    "35_arc_rail_staggered_gap.step": "32_arc_rail_staggered_gap.step",
    "36_arc_rail_three_body_chain.step": "33_arc_rail_three_body_chain.step",
    "37_v_joint_open_angle.step": "34_v_joint_open_angle.step",
    "38_stepped_blocks_multi_height.step": "35_stepped_blocks_multi_height.step",
    "39_box_box_overlap_1mm.step": "36_box_box_overlap_1mm.step",
    "40_box_on_cylinder_saddle_overlap.step": "37_box_on_cylinder_saddle_overlap.step",
    "41_two_plates_cross_overlap.step": "38_two_plates_cross_overlap.step",
    "32_angled_plate_gap_3mm.step": "39_angled_plate_gap_3mm.step",
    "16_angled_butt_joint.step": "45_angled_butt_joint.step",
    "08_cylinder_in_hole_clearance.step": "47_cylinder_in_hole_clearance.step",
    "11_cone_on_plate.step": "50_cone_on_plate.step",
    "28_cylinder_in_hole_clearance_3mm.step": "55_cylinder_in_hole_clearance_3mm.step",
    "30_cone_on_plate_gap_2mm.step": "57_cone_on_plate_gap_2mm.step",
}

# ─── Yardımcı fonksiyonlar ────────────────────────────────────────────────────

def save_step(filename, *shapes):
    """Birden fazla solid'i tek STEP dosyasına yaz (her biri ayrı body)."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    filename = SAVE_NAME_OVERRIDES.get(filename, filename)
    out_dir = os.path.dirname(os.path.join(OUTPUT_DIR, filename))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    filepath = os.path.join(OUTPUT_DIR, filename)
    writer = STEPControl_Writer()
    for shape in shapes:
        writer.Transfer(shape, STEPControl_AsIs)
    ok = writer.Write(filepath) == IFSelect_RetDone
    status = "OK  " if ok else "FAIL"
    print(f"  [{status}]  {filename}")
    return ok


def clean_output_steps():
    """Klasorleri koruyarak mevcut STEP/STP dosyalarini temizle."""
    if not os.path.isdir(OUTPUT_DIR):
        return
    for root, _, files in os.walk(OUTPUT_DIR):
        for name in files:
            if name.lower().endswith((".step", ".stp")):
                os.remove(os.path.join(root, name))


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


def annular_sector(r_inner, r_outer, angle_deg, thickness,
                   x=0.0, y=0.0, z=0.0, start_deg=0.0):
    """XY duzleminde halka dilimi olustur ve Z yonunde kalinlik ver."""
    a0 = math.radians(start_deg)
    a1 = math.radians(start_deg + angle_deg)
    am = (a0 + a1) / 2.0

    def pt(radius, angle):
        return gp_Pnt(x + radius * math.cos(angle),
                      y + radius * math.sin(angle),
                      z)

    outer0 = pt(r_outer, a0)
    outerm = pt(r_outer, am)
    outer1 = pt(r_outer, a1)
    inner0 = pt(r_inner, a0)
    innerm = pt(r_inner, am)
    inner1 = pt(r_inner, a1)

    outer_arc = BRepBuilderAPI_MakeEdge(
        GC_MakeArcOfCircle(outer0, outerm, outer1).Value()
    ).Edge()
    radial_end = BRepBuilderAPI_MakeEdge(outer1, inner1).Edge()
    inner_arc = BRepBuilderAPI_MakeEdge(
        GC_MakeArcOfCircle(inner1, innerm, inner0).Value()
    ).Edge()
    radial_start = BRepBuilderAPI_MakeEdge(inner0, outer0).Edge()

    wire = BRepBuilderAPI_MakeWire(
        outer_arc, radial_end, inner_arc, radial_start
    ).Wire()
    face = BRepBuilderAPI_MakeFace(wire).Face()
    return BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, thickness)).Shape()


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


def rotate_x(shape, angle_deg, cx=0.0, cy=0.0, cz=0.0):
    """Rotate shape around X axis through (cx, cy, cz)."""
    trsf = gp_Trsf()
    ax = gp_Ax1(gp_Pnt(cx, cy, cz), gp_Dir(1, 0, 0))
    trsf.SetRotation(ax, math.radians(angle_deg))
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def rotate_y(shape, angle_deg, cx=0.0, cy=0.0, cz=0.0):
    """Rotate shape around Y axis through (cx, cy, cz)."""
    trsf = gp_Trsf()
    ax = gp_Ax1(gp_Pnt(cx, cy, cz), gp_Dir(0, 1, 0))
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
    save_step("39_box_box_overlap_1mm.step", a, b)


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
    save_step("40_box_on_cylinder_saddle_overlap.step", cyl, blk)


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
    save_step("41_two_plates_cross_overlap.step", plate_a, plate_b)


# ─── Ana program ──────────────────────────────────────────────────────────────

def t19_box_box_gap_0p5mm():
    """Very small face-to-face gap (0.5 mm)."""
    a = box(50, 30, 20)
    b = box(50, 30, 20, z=20.5)
    save_step("19_box_box_gap_0p5mm.step", a, b)


def t20_box_box_gap_4p8mm():
    """Near-threshold gap (4.8 mm), still inside automatic proposal range."""
    a = box(50, 30, 20)
    b = box(50, 30, 20, z=24.8)
    save_step("20_box_box_gap_4p8mm.step", a, b)


def t21_box_box_gap_6mm_too_far():
    """Gap outside threshold (6 mm). Expected: too-far dialog."""
    a = box(50, 30, 20)
    b = box(50, 30, 20, z=26.0)
    save_step("21_box_box_gap_6mm_too_far.step", a, b)


def t22_box_box_corner_touch_only():
    """Only one corner point touches. Expected: no shared edge."""
    a = box(30, 30, 30)
    b = box(20, 20, 20, x=30, y=30, z=30)
    save_step("22_box_box_corner_touch_only.step", a, b)


def t23_box_box_edge_touch_only():
    """Bodies touch along one edge only."""
    a = box(40, 40, 20)
    b = box(20, 20, 20, x=40, y=40, z=0)
    save_step("23_box_box_edge_touch_only.step", a, b)


def t24_box_box_side_gap_3mm():
    """Two boxes facing each other through a side gap."""
    a = box(30, 50, 25)
    b = box(30, 50, 25, x=33)
    save_step("24_box_box_side_gap_3mm.step", a, b)


def t25_box_box_partial_side_gap_3mm():
    """Side gap with partial overlap in Y."""
    a = box(30, 60, 25)
    b = box(30, 30, 25, x=33, y=15)
    save_step("25_box_box_partial_side_gap_3mm.step", a, b)


def t26_box_box_offset_gap_3mm():
    """Z-gap plus XY offset: projected faces overlap partly."""
    a = box(60, 40, 20)
    b = box(40, 30, 20, x=10, y=5, z=23)
    save_step("26_box_box_offset_gap_3mm.step", a, b)


def t27_three_boxes_stacked_two_gaps():
    """Three separate bodies with two 2 mm gaps."""
    a = box(45, 30, 15)
    b = box(45, 30, 15, z=17)
    c = box(45, 30, 15, z=34)
    save_step("27_three_boxes_stacked_two_gaps.step", a, b, c)


def t28_cylinder_in_hole_large_clearance_3mm():
    """Cylinder in hole with 3 mm radial clearance."""
    block = box(70, 70, 22)
    drill = cylinder(r=16.0, h=26, x=35, y=35, z=-2)
    block_hole = cut(block, drill)
    pin = cylinder(r=13.0, h=34, x=35, y=35, z=-6)
    save_step("28_cylinder_in_hole_clearance_3mm.step", block_hole, pin)


def t29_cylinder_on_plate_gap_2mm():
    """Cylinder hovering 2 mm above a plate."""
    plate = box(100, 100, 10)
    cyl = cylinder(r=15, h=35, x=50, y=50, z=12)
    save_step("29_cylinder_on_plate_gap_2mm.step", plate, cyl)


def t30_cone_on_plate_gap_2mm():
    """Cone hovering 2 mm above a plate."""
    plate = box(120, 120, 10)
    cn = cone(r_base=20, r_top=0, h=40, x=60, y=60, z=12)
    save_step("30_cone_on_plate_gap_2mm.step", plate, cn)


def t31_t_joint_gap_2mm():
    """T-joint vertical plate lifted by 2 mm."""
    base = box(80, 60, 10)
    vertical = box(80, 8, 40, y=26, z=12)
    save_step("31_t_joint_gap_2mm.step", base, vertical)


def t32_angled_plate_gap_3mm():
    """Angled plate above base with a 3 mm gap."""
    base = box(90, 60, 8)
    plate = box(70, 12, 35, x=10, y=24, z=11)
    plate = rotate_z(plate, 18, cx=45, cy=30, cz=11)
    save_step("32_angled_plate_gap_3mm.step", base, plate)


def t33_arc_rail_radial_touch():
    """Two annular-sector rails sharing a curved radial seam."""
    inner = annular_sector(20, 30, 105, 8, x=0, y=0, z=0, start_deg=15)
    outer = annular_sector(30, 42, 105, 8, x=0, y=0, z=0, start_deg=15)
    save_step("33_arc_rail_radial_touch.step", inner, outer)


def t34_arc_rail_radial_gap_2mm():
    """Two annular-sector rails separated by a 2 mm curved gap."""
    inner = annular_sector(20, 30, 105, 8, x=0, y=0, z=0, start_deg=15)
    outer = annular_sector(32, 44, 105, 8, x=0, y=0, z=0, start_deg=15)
    save_step("34_arc_rail_radial_gap_2mm.step", inner, outer)


def t35_arc_rail_staggered_gap():
    """Curved rails with partial angular overlap and a radial gap."""
    inner = annular_sector(18, 30, 120, 8, x=0, y=0, z=0, start_deg=0)
    outer = annular_sector(33, 45, 80, 8, x=0, y=0, z=0, start_deg=25)
    save_step("35_arc_rail_staggered_gap.step", inner, outer)


def t36_arc_rail_three_body_chain():
    """Three curved rail bodies: two curved seams in one STEP file."""
    a = annular_sector(15, 25, 95, 8, x=0, y=0, z=0, start_deg=10)
    b = annular_sector(25, 35, 95, 8, x=0, y=0, z=0, start_deg=10)
    c = annular_sector(37, 47, 95, 8, x=0, y=0, z=0, start_deg=10)
    save_step("36_arc_rail_three_body_chain.step", a, b, c)


def t37_v_joint_open_angle():
    """Two plates forming a V-joint with an open angled seam."""
    left = box(70, 12, 35, x=-35, y=-6, z=0)
    right = box(70, 12, 35, x=-35, y=-6, z=0)
    left = rotate_z(left, -18, cx=0, cy=0, cz=0)
    right = rotate_z(right, 18, cx=0, cy=0, cz=0)
    right = translate(right, 0, 3, 0)
    save_step("37_v_joint_open_angle.step", left, right)


def t38_stepped_blocks_multi_height():
    """Three stepped blocks with different heights and two candidate seams."""
    a = box(35, 40, 12, x=0, y=0, z=0)
    b = box(35, 40, 22, x=35, y=0, z=0)
    c = box(35, 40, 16, x=70, y=0, z=0)
    save_step("38_stepped_blocks_multi_height.step", a, b, c)


TESTS = [
    # OKEY klasoru: referans / beklendigi gibi davranan testler
    ("OKEY/01  Box-Box flat touching", t01_box_box_flat_touching),
    ("OKEY/02.A Box-Box 3mm gap", t03_box_box_gap_3mm),
    ("OKEY/02.B Box-Box 0.5mm gap", t19_box_box_gap_0p5mm),
    ("OKEY/02.C Box-Box 4.8mm gap", t20_box_box_gap_4p8mm),
    ("OKEY/02.D Box-Box 6mm too far", t21_box_box_gap_6mm_too_far),
    ("OKEY/2.E Three boxes, two gaps", t27_three_boxes_stacked_two_gaps),
    ("OKEY/03  Box-Box partial shifted", t04_box_box_partial_shifted),
    ("OKEY/04  T-joint", t05_t_joint),
    ("OKEY/05  L-joint", t06_l_joint),
    ("OKEY/06  Cylinder on plate", t07_cylinder_on_plate),
    ("OKEY/08  Coaxial cylinders reducer", t09_coaxial_cylinders_reducer),
    ("OKEY/09  Cylinders side tangent", t10_cylinder_cylinder_side_tangent),
    ("OKEY/10  Sphere on plate", t12_sphere_on_plate),
    ("OKEY/11  Sphere-sphere tangent", t13_sphere_sphere_tangent),
    ("OKEY/12  Flange-to-flange", t14_flange_to_flange),
    ("OKEY/13  Torus on plate", t15_torus_on_plate),
    ("OKEY/14  Box-Box corner touch", t22_box_box_corner_touch_only),
    ("OKEY/15  Box-Box edge touch", t23_box_box_edge_touch_only),
    ("OKEY/30  Arc rail radial touch", t33_arc_rail_radial_touch),

    # Root klasor: zor / manuel / deneysel / problemli grup
    ("16  Partial side gap 3mm", t25_box_box_partial_side_gap_3mm),
    ("23  Offset Z-gap 3mm", t26_box_box_offset_gap_3mm),
    ("26  Cylinder on plate 2mm gap", t29_cylinder_on_plate_gap_2mm),
    ("28  T-joint 2mm gap", t31_t_joint_gap_2mm),
    ("31  Arc rail radial gap 2mm", t34_arc_rail_radial_gap_2mm),
    ("32  Arc rail staggered gap", t35_arc_rail_staggered_gap),
    ("33  Arc rail three-body chain", t36_arc_rail_three_body_chain),
    ("34  V-joint open angle", t37_v_joint_open_angle),
    ("35  Stepped blocks multi-height", t38_stepped_blocks_multi_height),
    ("36  Box-Box 1mm overlap", t02_box_box_overlap_1mm),
    ("37  Box on cylinder saddle overlap", t17_box_on_cylinder_saddle),
    ("38  Two plates cross overlap", t18_two_plates_overlapping_cross),
    ("39  Angled plate 3mm gap", t32_angled_plate_gap_3mm),
    ("45  Angled butt joint", t16_angled_butt_joint),
    ("47  Cylinder in hole clearance", t08_cylinder_in_hole_clearance),
    ("50  Cone on plate", t11_cone_on_plate),
    ("55  Cylinder in hole 3mm clearance", t28_cylinder_in_hole_large_clearance_3mm),
    ("57  Cone on plate 2mm gap", t30_cone_on_plate_gap_2mm),
]


if __name__ == "__main__":
    print("=" * 65)
    print("  Welding Test Geometry Generator")
    print(f"  Çıktı: {OUTPUT_DIR}")
    print("=" * 65)
    print()

    clean_output_steps()

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
    print("  Test sirasi onerisi:")
    print("  1. 01, 04, 05 -> baseline touching/shared-edge")
    print("  2. 02, 16, 17, 21 -> planar gap detection")
    print("  3. 19, 20 -> point/edge-only graceful handling")
    print("  4. 30-33 -> arc/yay yapilari")
    print("  5. 34, 35 -> ek karmasik multi-body/angled yapilar")
    print("  6. 36-38 -> overlap/cross/intersection testleri (en son)")
