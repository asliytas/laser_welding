"""
Larger robotic laser welding test geometry generator.

This script reuses the existing test set from generate_test_geometries.py and
writes uniformly scaled STEP files into:

    phase1/test_geometries/BIGGER

Usage:
    python bigger_generate_test_geometries.py
"""

import os

from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.gp import gp_Pnt, gp_Trsf
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Writer

import Desktop.welding_project.phase1.test_geometries.generate_test_geometries as base


SCALE_FACTOR = 30.0
OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "test_geometries",
    "BIGGER",
)


def scale_shape(shape, factor=SCALE_FACTOR):
    """Scale a shape uniformly from the global origin."""
    trsf = gp_Trsf()
    trsf.SetScale(gp_Pnt(0, 0, 0), factor)
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


def save_step(filename, *shapes):
    """Write scaled bodies into the BIGGER test geometry folder."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    filename = base.SAVE_NAME_OVERRIDES.get(filename, filename)

    filepath = os.path.join(OUTPUT_DIR, filename)
    out_dir = os.path.dirname(filepath)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    writer = STEPControl_Writer()
    for shape in shapes:
        writer.Transfer(scale_shape(shape), STEPControl_AsIs)

    ok = writer.Write(filepath) == IFSelect_RetDone
    status = "OK  " if ok else "FAIL"
    print(f"  [{status}]  {filename}")
    return ok


def clean_output_steps():
    """Keep folders, remove existing STEP/STP files from the BIGGER output."""
    if not os.path.isdir(OUTPUT_DIR):
        return

    for root, _, files in os.walk(OUTPUT_DIR):
        for name in files:
            if name.lower().endswith((".step", ".stp")):
                os.remove(os.path.join(root, name))


def run():
    """Generate the larger version of every existing test geometry."""
    original_save_step = base.save_step
    original_output_dir = base.OUTPUT_DIR

    base.save_step = save_step
    base.OUTPUT_DIR = OUTPUT_DIR

    try:
        print("=" * 72)
        print("  Bigger Welding Test Geometry Generator")
        print(f"  Scale factor: {SCALE_FACTOR:g}x")
        print(f"  Output: {OUTPUT_DIR}")
        print("=" * 72)
        print()

        clean_output_steps()

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
        print(f"  Result: {passed} successful / {failed} failed / {len(base.TESTS)} total")
        print("=" * 72)
        print()
        print("  Note: Dimensions, positions, gaps, and clearances are all scaled together.")
    finally:
        base.save_step = original_save_step
        base.OUTPUT_DIR = original_output_dir


if __name__ == "__main__":
    run()
