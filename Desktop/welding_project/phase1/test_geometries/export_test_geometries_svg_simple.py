"""
Pure-Python poster SVG exporter for welding test geometries.

It mirrors the dimensions used in generate_test_geometries.py, but draws clean
isometric schematics without requiring pythonocc/OCC. The output is meant for
figures/posters, not as a geometric substitute for the STEP files.

Usage:
    python export_test_geometries_svg_simple.py
"""

import math
import os


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "svg")
W, H = 900, 650
MARGIN = 58

COLORS = [
    ("#2563eb", "#dbeafe"),
    ("#dc2626", "#fee2e2"),
    ("#16a34a", "#dcfce7"),
    ("#9333ea", "#f3e8ff"),
]

SAVE_NAME_OVERRIDES = {
    "01_box_box_flat_touching.step": "OKEY/01_box_box_flat_touching.svg",
    "03_box_box_gap_3mm.step": "OKEY/02.A_box_box_gap_3mm.svg",
    "19_box_box_gap_0p5mm.step": "OKEY/02.B_box_box_gap_0p5mm.svg",
    "20_box_box_gap_4p8mm.step": "OKEY/02.C_box_box_gap_4p8mm.svg",
    "21_box_box_gap_6mm_too_far.step": "OKEY/02.D_box_box_gap_6mm_too_far.svg",
    "27_three_boxes_stacked_two_gaps.step": "OKEY/2.E_three_boxes_stacked_two_gaps.svg",
    "04_box_box_partial_shifted.step": "OKEY/03_box_box_partial_shifted.svg",
    "05_t_joint.step": "OKEY/04_t_joint.svg",
    "06_l_joint.step": "OKEY/05_l_joint.svg",
    "07_cylinder_on_plate.step": "OKEY/06_cylinder_on_plate.svg",
    "09_coaxial_cylinders_reducer.step": "OKEY/08_coaxial_cylinders_reducer.svg",
    "10_cylinder_cylinder_side_tangent.step": "OKEY/09_cylinder_cylinder_side_tangent.svg",
    "12_sphere_on_plate.step": "OKEY/10_sphere_on_plate.svg",
    "13_sphere_sphere_tangent.step": "OKEY/11_sphere_sphere_tangent.svg",
    "14_flange_to_flange.step": "OKEY/12_flange_to_flange.svg",
    "15_torus_on_plate.step": "OKEY/13_torus_on_plate.svg",
    "22_box_box_corner_touch_only.step": "OKEY/14_box_box_corner_touch_only.svg",
    "23_box_box_edge_touch_only.step": "OKEY/15_box_box_edge_touch_only.svg",
    "33_arc_rail_radial_touch.step": "OKEY/30_arc_rail_radial_touch.svg",
    "25_box_box_partial_side_gap_3mm.step": "16_box_box_partial_side_gap_3mm.svg",
    "26_box_box_offset_gap_3mm.step": "23_box_box_offset_gap_3mm.svg",
    "29_cylinder_on_plate_gap_2mm.step": "26_cylinder_on_plate_gap_2mm.svg",
    "31_t_joint_gap_2mm.step": "28_t_joint_gap_2mm.svg",
    "34_arc_rail_radial_gap_2mm.step": "31_arc_rail_radial_gap_2mm.svg",
    "35_arc_rail_staggered_gap.step": "32_arc_rail_staggered_gap.svg",
    "36_arc_rail_three_body_chain.step": "33_arc_rail_three_body_chain.svg",
    "37_v_joint_open_angle.step": "34_v_joint_open_angle.svg",
    "38_stepped_blocks_multi_height.step": "35_stepped_blocks_multi_height.svg",
    "39_box_box_overlap_1mm.step": "36_box_box_overlap_1mm.svg",
    "40_box_on_cylinder_saddle_overlap.step": "37_box_on_cylinder_saddle_overlap.svg",
    "41_two_plates_cross_overlap.step": "38_two_plates_cross_overlap.svg",
    "32_angled_plate_gap_3mm.step": "39_angled_plate_gap_3mm.svg",
    "16_angled_butt_joint.step": "45_angled_butt_joint.svg",
    "08_cylinder_in_hole_clearance.step": "47_cylinder_in_hole_clearance.svg",
    "11_cone_on_plate.step": "50_cone_on_plate.svg",
    "28_cylinder_in_hole_clearance_3mm.step": "55_cylinder_in_hole_clearance_3mm.svg",
    "30_cone_on_plate_gap_2mm.step": "57_cone_on_plate_gap_2mm.svg",
}


def iso(p):
    x, y, z = p
    return (x - 0.58 * y, -z + 0.36 * y)


def rotate_xy(p, angle_deg, cx=0.0, cy=0.0):
    x, y, z = p
    a = math.radians(angle_deg)
    dx, dy = x - cx, y - cy
    return (
        cx + dx * math.cos(a) - dy * math.sin(a),
        cy + dx * math.sin(a) + dy * math.cos(a),
        z,
    )


class Scene:
    def __init__(self, title):
        self.title = title
        self.items = []
        self.points = []

    def add_poly(self, pts, stroke, fill="none", width=2.2, opacity=1.0):
        self.items.append(("poly", pts, stroke, fill, width, opacity))
        self.points.extend(pts)

    def add_line(self, p1, p2, stroke, width=2.2, dash=None):
        self.items.append(("line", [p1, p2], stroke, dash or "", width, 1.0))
        self.points.extend([p1, p2])

    def bounds(self):
        if not self.points:
            return -1, 1, -1, 1
        coords = [iso(p) for p in self.points]
        xs, ys = zip(*coords)
        return min(xs), max(xs), min(ys), max(ys)

    def transform(self, p):
        min_x, max_x, min_y, max_y = self.bounds()
        sx = max(max_x - min_x, 1e-6)
        sy = max(max_y - min_y, 1e-6)
        scale = min((W - 2 * MARGIN) / sx, (H - 2 * MARGIN) / sy)
        ox = (W - sx * scale) * 0.5
        oy = (H - sy * scale) * 0.5 + 12
        x, y = iso(p)
        return ox + (x - min_x) * scale, oy + (y - min_y) * scale

    def svg(self):
        out = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
            '<rect width="100%" height="100%" fill="#ffffff"/>',
            f'<text x="28" y="38" font-family="Inter,Arial,sans-serif" font-size="22" font-weight="700" fill="#111827">{self.title}</text>',
        ]
        for kind, pts, stroke, fill_or_dash, width, opacity in self.items:
            if kind == "poly":
                coords = " ".join(f"{self.transform(p)[0]:.2f},{self.transform(p)[1]:.2f}" for p in pts)
                out.append(f'<polygon points="{coords}" fill="{fill_or_dash}" stroke="{stroke}" stroke-width="{width}" opacity="{opacity}" stroke-linejoin="round"/>')
            else:
                p1, p2 = pts
                x1, y1 = self.transform(p1)
                x2, y2 = self.transform(p2)
                dash = f' stroke-dasharray="{fill_or_dash}"' if fill_or_dash else ""
                out.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{stroke}" stroke-width="{width}" stroke-linecap="round"{dash}/>')
        out.append('<text x="28" y="622" font-family="Inter,Arial,sans-serif" font-size="14" fill="#64748b">schematic isometric SVG - bodies color coded</text>')
        out.append("</svg>")
        return "\n".join(out)


def box(scene, dx, dy, dz, x=0, y=0, z=0, color_idx=0, rot=0, cx=0, cy=0):
    stroke, fill = COLORS[color_idx % len(COLORS)]
    pts = {
        "000": (x, y, z), "100": (x + dx, y, z), "110": (x + dx, y + dy, z), "010": (x, y + dy, z),
        "001": (x, y, z + dz), "101": (x + dx, y, z + dz), "111": (x + dx, y + dy, z + dz), "011": (x, y + dy, z + dz),
    }
    if rot:
        pts = {k: rotate_xy(v, rot, cx, cy) for k, v in pts.items()}
    scene.add_poly([pts["001"], pts["101"], pts["111"], pts["011"]], stroke, fill, opacity=0.78)
    scene.add_poly([pts["000"], pts["100"], pts["101"], pts["001"]], stroke, "#ffffff", opacity=0.55)
    scene.add_poly([pts["100"], pts["110"], pts["111"], pts["101"]], stroke, "#ffffff", opacity=0.45)
    for a, b in [("000", "100"), ("100", "110"), ("110", "010"), ("010", "000"), ("001", "101"), ("101", "111"), ("111", "011"), ("011", "001"), ("000", "001"), ("100", "101"), ("110", "111"), ("010", "011")]:
        scene.add_line(pts[a], pts[b], stroke, 2.0)


def ellipse_ring(scene, x, y, z, r1, r2=None, color_idx=0, label=False):
    stroke, _ = COLORS[color_idx % len(COLORS)]
    pts = []
    for i in range(96):
        a = 2 * math.pi * i / 96
        pts.append((x + r1 * math.cos(a), y + r1 * math.sin(a), z))
    scene.add_poly(pts, stroke, "none", 2.4)
    if r2:
        pts2 = []
        for i in range(96):
            a = 2 * math.pi * i / 96
            pts2.append((x + r2 * math.cos(a), y + r2 * math.sin(a), z))
        scene.add_poly(pts2, stroke, "none", 2.0)


def cylinder_z(scene, r, h, x, y, z, color_idx=0):
    stroke, fill = COLORS[color_idx % len(COLORS)]
    ellipse_ring(scene, x, y, z + h, r, color_idx=color_idx)
    ellipse_ring(scene, x, y, z, r, color_idx=color_idx)
    for a in [0, math.pi]:
        scene.add_line((x + r * math.cos(a), y + r * math.sin(a), z), (x + r * math.cos(a), y + r * math.sin(a), z + h), stroke)


def sphere(scene, r, x, y, z, color_idx=0):
    ellipse_ring(scene, x, y, z, r, color_idx=color_idx)
    ellipse_ring(scene, x, y, z, r * 0.58, color_idx=color_idx)


def cone(scene, r, h, x, y, z, color_idx=0):
    stroke, _ = COLORS[color_idx % len(COLORS)]
    ellipse_ring(scene, x, y, z, r, color_idx=color_idx)
    tip = (x, y, z + h)
    scene.add_line((x - r, y, z), tip, stroke)
    scene.add_line((x + r, y, z), tip, stroke)


def annular(scene, r1, r2, start, angle, z, color_idx=0):
    stroke, _ = COLORS[color_idx % len(COLORS)]
    pts = []
    for i in range(64):
        a = math.radians(start + angle * i / 63)
        pts.append((r2 * math.cos(a), r2 * math.sin(a), z))
    for i in reversed(range(64)):
        a = math.radians(start + angle * i / 63)
        pts.append((r1 * math.cos(a), r1 * math.sin(a), z))
    scene.add_poly(pts, stroke, "none", 2.5)


def title_from(filename):
    rel = SAVE_NAME_OVERRIDES.get(filename, filename)
    return os.path.splitext(os.path.basename(rel))[0].replace("_", " ")


def write_scene(filename, scene):
    rel = SAVE_NAME_OVERRIDES.get(filename, filename).replace(".step", ".svg")
    out = os.path.join(OUTPUT_DIR, rel)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(scene.svg())
    print("  [SVG ]", rel)


def make(filename, builder):
    s = Scene(title_from(filename))
    builder(s)
    write_scene(filename, s)


def run():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    for root, _, files in os.walk(OUTPUT_DIR):
        for name in files:
            if name.endswith(".svg"):
                os.remove(os.path.join(root, name))

    tests = [
        ("01_box_box_flat_touching.step", lambda s: (box(s, 50, 30, 20, 0, 0, 0, 0), box(s, 50, 30, 20, 0, 0, 20, 1))),
        ("03_box_box_gap_3mm.step", lambda s: (box(s, 50, 30, 20, 0, 0, 0, 0), box(s, 50, 30, 20, 0, 0, 23, 1))),
        ("19_box_box_gap_0p5mm.step", lambda s: (box(s, 50, 30, 20, 0, 0, 0, 0), box(s, 50, 30, 20, 0, 0, 20.5, 1))),
        ("20_box_box_gap_4p8mm.step", lambda s: (box(s, 50, 30, 20, 0, 0, 0, 0), box(s, 50, 30, 20, 0, 0, 24.8, 1))),
        ("21_box_box_gap_6mm_too_far.step", lambda s: (box(s, 50, 30, 20, 0, 0, 0, 0), box(s, 50, 30, 20, 0, 0, 26, 1))),
        ("27_three_boxes_stacked_two_gaps.step", lambda s: (box(s, 45, 30, 15, 0, 0, 0, 0), box(s, 45, 30, 15, 0, 0, 17, 1), box(s, 45, 30, 15, 0, 0, 34, 2))),
        ("04_box_box_partial_shifted.step", lambda s: (box(s, 60, 40, 20, 0, 0, 0, 0), box(s, 60, 40, 20, 30, 0, 20, 1))),
        ("05_t_joint.step", lambda s: (box(s, 80, 60, 10, 0, 0, 0, 0), box(s, 80, 8, 40, 0, 26, 10, 1))),
        ("06_l_joint.step", lambda s: (box(s, 80, 60, 8, 0, 0, 0, 0), box(s, 8, 60, 50, 80, 0, 0, 1))),
        ("07_cylinder_on_plate.step", lambda s: (box(s, 100, 100, 10, 0, 0, 0, 0), cylinder_z(s, 15, 40, 50, 50, 10, 1))),
        ("09_coaxial_cylinders_reducer.step", lambda s: (cylinder_z(s, 20, 30, 50, 50, 0, 0), cylinder_z(s, 12, 30, 50, 50, 30, 1))),
        ("10_cylinder_cylinder_side_tangent.step", lambda s: (cylinder_z(s, 15, 60, 15, 30, 0, 0), cylinder_z(s, 15, 60, 45, 30, 0, 1))),
        ("12_sphere_on_plate.step", lambda s: (box(s, 100, 100, 10, 0, 0, 0, 0), sphere(s, 25, 50, 50, 35, 1))),
        ("13_sphere_sphere_tangent.step", lambda s: (sphere(s, 20, 20, 50, 50, 0), sphere(s, 20, 60, 50, 50, 1))),
        ("14_flange_to_flange.step", lambda s: (cylinder_z(s, 40, 8, 0, 0, 0, 0), ellipse_ring(s, 0, 0, 8, 15, color_idx=0), cylinder_z(s, 40, 8, 0, 0, 8, 1))),
        ("15_torus_on_plate.step", lambda s: (box(s, 120, 120, 10, 0, 0, 0, 0), ellipse_ring(s, 60, 60, 18, 33, 17, 1))),
        ("22_box_box_corner_touch_only.step", lambda s: (box(s, 30, 30, 30, 0, 0, 0, 0), box(s, 20, 20, 20, 30, 30, 30, 1))),
        ("23_box_box_edge_touch_only.step", lambda s: (box(s, 40, 40, 20, 0, 0, 0, 0), box(s, 20, 20, 20, 40, 40, 0, 1))),
        ("33_arc_rail_radial_touch.step", lambda s: (annular(s, 20, 30, 15, 105, 0, 0), annular(s, 30, 42, 15, 105, 0, 1))),
        ("25_box_box_partial_side_gap_3mm.step", lambda s: (box(s, 30, 60, 25, 0, 0, 0, 0), box(s, 30, 30, 25, 33, 15, 0, 1))),
        ("26_box_box_offset_gap_3mm.step", lambda s: (box(s, 60, 40, 20, 0, 0, 0, 0), box(s, 40, 30, 20, 10, 5, 23, 1))),
        ("29_cylinder_on_plate_gap_2mm.step", lambda s: (box(s, 100, 100, 10, 0, 0, 0, 0), cylinder_z(s, 15, 35, 50, 50, 12, 1))),
        ("31_t_joint_gap_2mm.step", lambda s: (box(s, 80, 60, 10, 0, 0, 0, 0), box(s, 80, 8, 40, 0, 26, 12, 1))),
        ("34_arc_rail_radial_gap_2mm.step", lambda s: (annular(s, 20, 30, 15, 105, 0, 0), annular(s, 32, 44, 15, 105, 0, 1))),
        ("35_arc_rail_staggered_gap.step", lambda s: (annular(s, 18, 30, 0, 120, 0, 0), annular(s, 33, 45, 25, 80, 0, 1))),
        ("36_arc_rail_three_body_chain.step", lambda s: (annular(s, 15, 25, 10, 95, 0, 0), annular(s, 25, 35, 10, 95, 0, 1), annular(s, 37, 47, 10, 95, 0, 2))),
        ("37_v_joint_open_angle.step", lambda s: (box(s, 70, 12, 35, -35, -6, 0, 0, -18, 0, 0), box(s, 70, 12, 35, -35, -3, 0, 1, 18, 0, 0))),
        ("38_stepped_blocks_multi_height.step", lambda s: (box(s, 35, 40, 12, 0, 0, 0, 0), box(s, 35, 40, 22, 35, 0, 0, 1), box(s, 35, 40, 16, 70, 0, 0, 2))),
        ("39_box_box_overlap_1mm.step", lambda s: (box(s, 50, 30, 20, 0, 0, 0, 0), box(s, 50, 30, 20, 0, 0, 19, 1))),
        ("40_box_on_cylinder_saddle_overlap.step", lambda s: (cylinder_z(s, 20, 100, 50, 30, 0, 0), box(s, 30, 30, 30, 35, 35, 48, 1))),
        ("41_two_plates_cross_overlap.step", lambda s: (box(s, 100, 10, 4, -50, -5, 0, 0), box(s, 100, 10, 4, -50, -5, 0, 1, 90, 0, 0))),
        ("32_angled_plate_gap_3mm.step", lambda s: (box(s, 90, 60, 8, 0, 0, 0, 0), box(s, 70, 12, 35, 10, 24, 11, 1, 18, 45, 30))),
        ("16_angled_butt_joint.step", lambda s: (box(s, 50, 30, 20, 0, 0, 0, 0), box(s, 50, 30, 20, 50, 0, 0, 1, 45, 75, 15))),
        ("08_cylinder_in_hole_clearance.step", lambda s: (box(s, 60, 60, 20, 0, 0, 0, 0), ellipse_ring(s, 30, 30, 20, 12, 11.8, 1))),
        ("11_cone_on_plate.step", lambda s: (box(s, 120, 120, 10, 0, 0, 0, 0), cone(s, 20, 40, 60, 60, 10, 1))),
        ("28_cylinder_in_hole_clearance_3mm.step", lambda s: (box(s, 70, 70, 22, 0, 0, 0, 0), ellipse_ring(s, 35, 35, 22, 16, 13, 1))),
        ("30_cone_on_plate_gap_2mm.step", lambda s: (box(s, 120, 120, 10, 0, 0, 0, 0), cone(s, 20, 40, 60, 60, 12, 1))),
    ]

    print("=" * 72)
    print("  Pure SVG Welding Test Geometry Exporter")
    print(f"  Output: {OUTPUT_DIR}")
    print("=" * 72)
    for filename, builder in tests:
        make(filename, builder)
    print("=" * 72)
    print(f"  Result: {len(tests)} SVG files")
    print("=" * 72)


if __name__ == "__main__":
    run()
