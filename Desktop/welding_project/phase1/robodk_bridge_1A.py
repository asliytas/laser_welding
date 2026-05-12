"""
RoboDK Bridge 1.A for the Welding Trajectory System.

Reads a `weld_program_1.A` JSON file (produced by `trajectory_planner_1A`)
and pushes it into a running RoboDK station as a `Program` item with
Targets, MoveJ/MoveL instructions, speed changes, and process I/O calls.

Usage
-----
    # CLI
    python robodk_bridge_1A.py --input weld.json [--program-name MyWeld]

    # Programmatic (from trajectory_planner_1A)
    from robodk_bridge_1A import load_to_robodk, RoboDKBridgeError
    load_to_robodk("weld.json", program_name="MyWeld")

Station prerequisites (the user is responsible for these):
    - A robot of any kind (auto-picked, or chosen interactively).
    - A frame named "WorkpieceFrame" (all JSON positions are relative to it).
    - A tool named "WeldingTorch" attached to the robot flange.

The bridge does NOT create or modify these items; if any is missing the
user is prompted to pick it (interactive=True) or the call aborts cleanly.
"""

import argparse
import json
import math
import os
import sys

# Defer robodk import so the module can be imported on machines without
# RoboDK installed (e.g. the planner host without RoboDK).
try:
    from robodk.robolink import (
        Robolink,
        ITEM_TYPE_ROBOT,
        ITEM_TYPE_FRAME,
        ITEM_TYPE_TOOL,
        INSTRUCTION_CALL_PROGRAM,
    )
    from robodk.robomath import Mat
    ROBODK_AVAILABLE = True
    ROBODK_IMPORT_ERROR = None
except Exception as exc:
    ROBODK_AVAILABLE = False
    ROBODK_IMPORT_ERROR = exc


# === Constants ===
DEFAULT_FRAME_NAME = "WorkpieceFrame"
DEFAULT_TOOL_NAME = "WeldingTorch"
SPEED_EPSILON_MM_S = 0.1
MIN_SPEED_MM_S = 1.0
POSE_CROSSCHECK_TOL = 0.01

VALID_PHASES = ("approach", "weld", "retract")


# === Custom exceptions ===
class RoboDKBridgeError(Exception):
    """Raised on any failure in the bridge layer (user-facing message)."""
    pass


# === JSON loading & validation ===
def _load_json(path):
    if not os.path.isfile(path):
        raise RoboDKBridgeError(f"JSON file not found: {path}")
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except json.JSONDecodeError as exc:
        raise RoboDKBridgeError(f"Invalid JSON in {path}: {exc}")

    if not isinstance(data, dict):
        raise RoboDKBridgeError("JSON root must be an object.")
    programs = data.get("programs")
    if not isinstance(programs, list) or not programs:
        raise RoboDKBridgeError("JSON is missing a non-empty 'programs' list.")

    for w_idx, weld in enumerate(programs):
        _validate_weld(weld, w_idx)
    return data


def _validate_weld(weld, idx):
    if not isinstance(weld, dict):
        raise RoboDKBridgeError(f"Weld #{idx} is not an object.")
    setpoints = weld.get("setpoints")
    if not isinstance(setpoints, list) or not setpoints:
        raise RoboDKBridgeError(f"Weld #{idx} has no setpoints.")
    for s_idx, sp in enumerate(setpoints):
        for key in ("index", "phase", "pose", "velocity"):
            if key not in sp:
                raise RoboDKBridgeError(
                    f"Weld #{idx} setpoint #{s_idx} missing '{key}'."
                )
        if sp["phase"] not in VALID_PHASES:
            raise RoboDKBridgeError(
                f"Weld #{idx} setpoint #{s_idx} has unknown phase '{sp['phase']}'."
            )
        pose = sp["pose"]
        pos = pose.get("position")
        rot = pose.get("rotation_matrix")
        if not (isinstance(pos, (list, tuple)) and len(pos) == 3):
            raise RoboDKBridgeError(
                f"Weld #{idx} setpoint #{s_idx} pose.position must be [x,y,z]."
            )
        if not (isinstance(rot, list) and len(rot) == 3
                and all(isinstance(r, list) and len(r) == 3 for r in rot)):
            raise RoboDKBridgeError(
                f"Weld #{idx} setpoint #{s_idx} pose.rotation_matrix must be 3x3."
            )


# === Pose construction ===
def _build_pose(setpoint):
    """Build a 4x4 RoboDK Mat from a setpoint's rotation_matrix + position.

    JSON convention: rotation_matrix is row-major; columns are tool X,Y,Z
    axes expressed in WorkpieceFrame coordinates. RoboDK's Mat takes the
    same row-major form, so the mapping is direct.
    """
    R = setpoint["pose"]["rotation_matrix"]
    p = setpoint["pose"]["position"]
    return Mat([
        [R[0][0], R[0][1], R[0][2], p[0]],
        [R[1][0], R[1][1], R[1][2], p[1]],
        [R[2][0], R[2][1], R[2][2], p[2]],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _crosscheck_pose(setpoint):
    """Optionally compare the rotation matrix to the quaternion. Returns
    a warning string if they disagree, else None. Never raises.
    """
    pose = setpoint["pose"]
    quat = pose.get("quaternion")
    R = pose.get("rotation_matrix")
    if not (isinstance(quat, (list, tuple)) and len(quat) == 4 and R):
        return None
    w, x, y, z = quat
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return None
    w, x, y, z = w / n, x / n, y / n, z / n
    R_q = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ]
    max_err = max(abs(R[i][j] - R_q[i][j]) for i in range(3) for j in range(3))
    if max_err > POSE_CROSSCHECK_TOL:
        return (f"setpoint {setpoint.get('index')}: rotation_matrix vs "
                f"quaternion mismatch (max delta {max_err:.4f})")
    return None


# === Station item resolution ===
def _resolve_station_items(RDK, frame_name, tool_name, interactive_fallback=True):
    """Return (robot, frame, tool); raise RoboDKBridgeError on failure."""
    robots = RDK.ItemList(ITEM_TYPE_ROBOT)
    if not robots:
        raise RoboDKBridgeError(
            "No robot found in the RoboDK station. Add a robot and try again."
        )
    if len(robots) == 1:
        robot = robots[0]
    else:
        if interactive_fallback:
            robot = RDK.ItemUserPick("Select robot", ITEM_TYPE_ROBOT)
            if not robot.Valid():
                raise RoboDKBridgeError("No robot selected; aborting.")
        else:
            robot = robots[0]

    frame = RDK.Item(frame_name, ITEM_TYPE_FRAME)
    if not frame.Valid():
        if interactive_fallback:
            frame = RDK.ItemUserPick(
                f"Select frame ('{frame_name}' not found)", ITEM_TYPE_FRAME
            )
        if not frame.Valid():
            raise RoboDKBridgeError(
                f"Frame '{frame_name}' not found and no frame selected. "
                f"Create a frame named '{frame_name}' in your RoboDK station."
            )

    tool = RDK.Item(tool_name, ITEM_TYPE_TOOL)
    if not tool.Valid():
        if interactive_fallback:
            tool = RDK.ItemUserPick(
                f"Select tool ('{tool_name}' not found)", ITEM_TYPE_TOOL
            )
        if not tool.Valid():
            raise RoboDKBridgeError(
                f"Tool '{tool_name}' not found and no tool selected. "
                f"Attach a tool named '{tool_name}' to the robot flange."
            )

    return robot, frame, tool


# === Phase transition I/O ===
def _phase_transition_io(program, prev_phase, new_phase):
    """Emit closing I/O for prev_phase, then opening I/O for new_phase.

    new_phase == None means end-of-program (only emit closing).
    prev_phase == None means start-of-program (only emit opening).
    """
    # Closing instructions for the phase we're leaving.
    if prev_phase == "weld":
        program.RunInstruction("LaserOff", INSTRUCTION_CALL_PROGRAM)
        program.RunInstruction("WireFeedStop", INSTRUCTION_CALL_PROGRAM)
    if prev_phase == "retract" and new_phase is None:
        program.RunInstruction("GasOff", INSTRUCTION_CALL_PROGRAM)
    # If a weld program has no retract phase, still close gas at program end.
    if prev_phase in ("approach", "weld") and new_phase is None:
        program.RunInstruction("GasOff", INSTRUCTION_CALL_PROGRAM)

    # Opening instructions for the phase we're entering.
    if new_phase == "approach":
        program.RunInstruction("GasOn", INSTRUCTION_CALL_PROGRAM)
    elif new_phase == "weld":
        program.RunInstruction("LaserOn", INSTRUCTION_CALL_PROGRAM)
        program.RunInstruction("WireFeedStart", INSTRUCTION_CALL_PROGRAM)


# === Program construction ===
def _build_program(RDK, robot, frame, tool, weld_data, program_name):
    """Build one Program item from a single weld's setpoints."""
    existing = RDK.Item(program_name)
    if existing.Valid():
        existing.Delete()

    program = RDK.AddProgram(program_name, robot)
    program.setFrame(frame)
    program.setTool(tool)

    setpoints = weld_data["setpoints"]
    current_speed = None
    prev_phase = None
    warnings = []

    for i, sp in enumerate(setpoints):
        if sp["phase"] != prev_phase:
            _phase_transition_io(program, prev_phase, sp["phase"])
            prev_phase = sp["phase"]
            is_first_of_phase = True
        else:
            is_first_of_phase = False

        # Optional sanity check (does not block).
        warn = _crosscheck_pose(sp)
        if warn:
            warnings.append(warn)

        # Build target as a child of the workpiece frame.
        pose = _build_pose(sp)
        target_name = f"{program_name}_p{int(sp['index']):04d}"
        target = RDK.AddTarget(target_name, frame, robot)
        target.setPose(pose)
        target.setAsCartesianTarget()

        # Speed change only on actual delta (or first point).
        v = float(sp["velocity"])
        if current_speed is None or abs(v - current_speed) > SPEED_EPSILON_MM_S:
            program.setSpeed(max(v, MIN_SPEED_MM_S))
            current_speed = v

        # MoveJ for the very first point of an approach phase; MoveL otherwise.
        if sp["phase"] == "approach" and is_first_of_phase:
            program.MoveJ(target)
        else:
            program.MoveL(target)

    # Close out the last phase (and emit GasOff at program end).
    _phase_transition_io(program, prev_phase, None)

    if warnings:
        # Log to RoboDK message panel; non-fatal.
        try:
            RDK.ShowMessage(
                f"[{program_name}] pose cross-check warnings: "
                + "; ".join(warnings[:5])
                + (f" (+{len(warnings) - 5} more)" if len(warnings) > 5 else ""),
                popup=False,
            )
        except Exception:
            pass

    return program


# === Main entry point ===
def load_to_robodk(
    json_path,
    program_name=None,
    frame_name=DEFAULT_FRAME_NAME,
    tool_name=DEFAULT_TOOL_NAME,
    interactive=True,
):
    """Load a weld_program_1.A JSON into RoboDK as one or more Program items.

    Returns: list of created Program items (one per weld).
    Raises:  RoboDKBridgeError on any failure (with user-facing message).
    """
    if not ROBODK_AVAILABLE:
        raise RoboDKBridgeError(
            "robodk package is not installed. Install with "
            "`pip install robodk`.\n"
            f"Underlying error: {ROBODK_IMPORT_ERROR}"
        )

    data = _load_json(json_path)

    # Connect to a running RoboDK instance. Robolink() raises if the app
    # is not running / not reachable.
    try:
        RDK = Robolink()
    except Exception as exc:
        raise RoboDKBridgeError(
            "Could not connect to RoboDK. Make sure RoboDK is running and "
            f"the station is open.\nUnderlying error: {exc}"
        )

    robot, frame, tool = _resolve_station_items(
        RDK, frame_name, tool_name, interactive_fallback=interactive
    )

    welds = data["programs"]
    multi = len(welds) > 1
    created = []
    sub_programs = []

    for weld in welds:
        weld_id = weld.get("weld_id", len(created) + 1)
        base = program_name or "Weld"
        prog_name = base if (not multi and program_name) else f"{base}_{weld_id}"
        prog = _build_program(RDK, robot, frame, tool, weld, prog_name)
        created.append(prog)
        sub_programs.append(prog_name)

    # When multiple welds are present, add a master program that calls each
    # sub-program in order. Idempotent — overwrite if it already exists.
    if multi:
        master_name = (program_name or "Weld") + "_All"
        existing = RDK.Item(master_name)
        if existing.Valid():
            existing.Delete()
        master = RDK.AddProgram(master_name, robot)
        master.setFrame(frame)
        master.setTool(tool)
        for sub in sub_programs:
            master.RunInstruction(sub, INSTRUCTION_CALL_PROGRAM)
        created.append(master)

    return created


# === CLI ===
def main(argv=None):
    parser = argparse.ArgumentParser(description="RoboDK Bridge 1.A")
    parser.add_argument("--input", required=True, help="Path to weld_program_1.A JSON")
    parser.add_argument("--program-name", default=None,
                        help="Base name for the RoboDK Program item(s)")
    parser.add_argument("--frame", default=DEFAULT_FRAME_NAME,
                        help=f"Frame name (default: {DEFAULT_FRAME_NAME})")
    parser.add_argument("--tool", default=DEFAULT_TOOL_NAME,
                        help=f"Tool name (default: {DEFAULT_TOOL_NAME})")
    parser.add_argument("--non-interactive", action="store_true",
                        help="Do not show RoboDK pick dialogs on missing items.")
    args = parser.parse_args(argv)

    try:
        progs = load_to_robodk(
            args.input,
            program_name=args.program_name,
            frame_name=args.frame,
            tool_name=args.tool,
            interactive=not args.non_interactive,
        )
        print(f"Loaded {len(progs)} program(s) into RoboDK:")
        for p in progs:
            try:
                print(f"  - {p.Name()}")
            except Exception:
                print("  - <program>")
        return 0
    except RoboDKBridgeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
