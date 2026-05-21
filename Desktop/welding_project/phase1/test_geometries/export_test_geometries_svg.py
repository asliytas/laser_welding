"""
Export poster-friendly SVG previews for the welding test geometries.

This script reuses generate_test_geometries.py, but monkeypatches save_step()
so each test writes a lightweight isometric SVG instead of a STEP file.

Usage:
    python export_test_geometries_svg.py
"""

import math
import os
import sys

from OCC.Core.BRep import BRep_Tool
from OCC.Core.GCPnts import GCPnts_AbscissaPoint
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Extend.TopologyUtils import TopologyExplorer


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import generate_test_geometries as base


OUTPUT_DIR = os.path.join(SCRIPT_DIR, "svg")
CANVAS_W = 900
CANVAS_H = 650
MARGIN = 52
EDGE_SAMPLES = 18

BODY_COLORS = [
    ("#4f7bd9", "#dce8ff"),
    ("#d9654f", "#ffe4dc"),
    ("#42a66a", "#ddf5e5"),
    ("#a45bd8", "#f0ddff"),
]


def _svg_name(filename):
    rel = base.SAVE_NAME_OVERRIDES.get(filename, filename)
    stem = os.path.splitext(rel)[0] + ".svg"
    return os.path.join(OUTPUT_DIR, stem)


def _point_tuple(p):
    return (p.X(), p.Y(), p.Z())


def _edge_points(edge, samples=EDGE_SAMPLES):
    try:
        curve_data = BRep_Tool.Curve(edge)
        if curve_data is None or len(curve_data) < 3:
            return []
        curve, u_min, u_max = curve_data[0], curve_data[-2], curve_data[-1]
        if curve is None:
            return []

        length = 0.0
        try:
            length = GCPnts_AbscissaPoint.Length(curve, u_min, u_max)
        except Exception:
            pass
        n = max(2, min(48, int(max(samples, length / 4.0))))

        pts = []
        for i in range(n):
            t = i / (n - 1)
            u = u_min + (u_max - u_min) * t
            pts.append(_point_tuple(curve.Value(u)))
        return pts
    except Exception:
        return []


def _shape_edges(shape):
    edges = []
    exp = TopExp_Explorer(shape, TopAbs_EDGE)
    while exp.More():
        pts = _edge_points(exp.Current())
        if len(pts) >= 2:
            edges.append(pts)
        exp.Next()
    return edges


def _project_iso(p):
    x, y, z = p
    # Quiet engineering isometric: enough depth for a poster without clutter.
    return (x - 0.55 * y, -z + 0.35 * y)


def _projected_bounds(edge_groups):
    xs, ys = [], []
    for edges in edge_groups:
        for edge in edges:
            for p in edge:
                x, y = _project_iso(p)
                xs.append(x)
                ys.append(y)
    if not xs:
        return (0.0, 1.0, 0.0, 1.0)
    return min(xs), max(xs), min(ys), max(ys)


def _transformer(edge_groups):
    min_x, max_x, min_y, max_y = _projected_bounds(edge_groups)
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    scale = min((CANVAS_W - 2 * MARGIN) / span_x, (CANVAS_H - 2 * MARGIN) / span_y)
    offset_x = (CANVAS_W - span_x * scale) * 0.5
    offset_y = (CANVAS_H - span_y * scale) * 0.5

    def tx(p):
        x, y = _project_iso(p)
        return (
            offset_x + (x - min_x) * scale,
            offset_y + (y - min_y) * scale,
        )

    return tx


def _polyline(points, transform, stroke, width=2.4):
    coords = []
    for p in points:
        x, y = transform(p)
        coords.append(f"{x:.2f},{y:.2f}")
    return (
        f'<polyline points="{" ".join(coords)}" fill="none" '
        f'stroke="{stroke}" stroke-width="{width}" '
        f'stroke-linecap="round" stroke-linejoin="round"/>'
    )


def _body_bounds_xy(shape):
    xs, ys = [], []
    try:
        for vertex in TopologyExplorer(shape).vertices():
            p = BRep_Tool.Pnt(vertex)
            xs.append(p.X())
            ys.append(p.Y())
    except Exception:
        return None
    if not xs:
        return None
    return min(xs), max(xs), min(ys), max(ys)


def _svg_label(filename):
    rel = base.SAVE_NAME_OVERRIDES.get(filename, filename)
    return os.path.splitext(os.path.basename(rel))[0].replace("_", " ")


def save_svg(filename, *shapes):
    edge_groups = [_shape_edges(shape) for shape in shapes]
    edge_groups = [edges for edges in edge_groups if edges]
    if not edge_groups:
        print(f"  [SKIP] {filename} has no drawable edges")
        return False

    out_path = _svg_name(filename)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    transform = _transformer(edge_groups)

    title = _svg_label(filename)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_W}" height="{CANVAS_H}" viewBox="0 0 {CANVAS_W} {CANVAS_H}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="28" y="38" font-family="Inter, Arial, sans-serif" font-size="22" font-weight="700" fill="#1f2937">{title}</text>',
        '<g opacity="0.98">',
    ]

    for body_idx, edges in enumerate(edge_groups):
        stroke, _ = BODY_COLORS[body_idx % len(BODY_COLORS)]
        for edge in edges:
            parts.append(_polyline(edge, transform, stroke))

    parts.extend([
        "</g>",
        '<text x="28" y="622" font-family="Inter, Arial, sans-serif" font-size="14" fill="#64748b">isometric SVG preview - bodies color coded</text>',
        "</svg>",
    ])

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    print(f"  [SVG ] {os.path.relpath(out_path, OUTPUT_DIR)}")
    return True


def clean_output_svgs():
    if not os.path.isdir(OUTPUT_DIR):
        return
    for root, _, files in os.walk(OUTPUT_DIR):
        for name in files:
            if name.lower().endswith(".svg"):
                os.remove(os.path.join(root, name))


def run():
    original_save_step = base.save_step
    try:
        base.save_step = save_svg
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        clean_output_svgs()

        print("=" * 72)
        print("  Welding Test Geometry SVG Exporter")
        print(f"  Output: {OUTPUT_DIR}")
        print("=" * 72)
        print()

        passed, failed = 0, 0
        for desc, fn in base.TESTS:
            print(f"  {desc}")
            try:
                fn()
                passed += 1
            except Exception as exc:
                print(f"         ERROR: {exc}")
                failed += 1
            print()

        print("=" * 72)
        print(f"  Result: {passed} SVG groups exported / {failed} failed / {len(base.TESTS)} total")
        print("=" * 72)
    finally:
        base.save_step = original_save_step


if __name__ == "__main__":
    run()
