"""
Verify that data/town01/LawXX_basic_info.json round-trips back to the real
traffic_rule_BehAVExplor/LawXX/objects.json coordinates - without running
any simulation. This exercises exactly the conversion path (basic_info.json
-> Scenario -> get_lgsvl_input() -> objects_writer.build_objects_json())
and nothing downstream of it (no mock_bridge, no frame.py oracle, no
fuzzing loop), so it's independent of mock_bridge.py's interpolation
quality - only the conversion itself is under test.

Auto-detects which data/<town>/ a scenario's basic_info.json lives in.

Usage: python3 tools/verify_traffic_rule_conversion.py Law1 Law2 Law27
       python3 tools/verify_traffic_rule_conversion.py --all
"""
import argparse
import glob
import json
import math
import os

from common.scenario import Scenario
from common import objects_writer

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(REPO_ROOT, "traffic_rule_BehAVExplor")


def _dist(a, b):
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def _find_basic_info(law_name):
    matches = glob.glob(os.path.join(REPO_ROOT, "data", "*", "%s_basic_info.json" % law_name))
    if not matches:
        raise RuntimeError("no converted basic_info.json found for %s" % law_name)
    return matches[0]


def verify(law_name):
    print("=== %s ===" % law_name)
    basic_info_path = _find_basic_info(law_name)
    town_dir = os.path.dirname(basic_info_path)
    town_slug = os.path.basename(town_dir)
    basic_info = json.load(open(basic_info_path))[0]
    route_info = json.load(open(os.path.join(town_dir, "%s_routes.json" % town_slug)))
    real = json.load(open(os.path.join(SRC_DIR, law_name, "objects.json")))

    scenario = Scenario()
    scenario.generate_specific_info_wo_npc(basic_info, route_info)
    synthetic_lanes = basic_info.get("synthetic_lanes")
    if synthetic_lanes:
        scenario.add_synthetic_lanes(synthetic_lanes)
    fixed_npc = basic_info.get("fixed_npc")
    if fixed_npc:
        scenario.set_fixed_npc_info(fixed_npc["types"], fixed_npc["routes"], fixed_npc["waypoints"])
    else:
        scenario.npc_types, scenario.npc_routes, scenario.npc_routes_ids, scenario.npc_waypoints = [], [], [], []

    objects_json = objects_writer.build_objects_json(scenario.get_lgsvl_input())

    real_ego = next(o for o in real["ego"] if "spawn_point" in o)
    gen_ego = next(o for o in objects_json["ego"] if "spawn_point" in o)
    d_spawn = _dist(real_ego["spawn_point"], gen_ego["spawn_point"])
    d_dest = _dist(real_ego["dest_point"], gen_ego["dest_point"])
    print("  ego spawn error: %.3fm" % d_spawn)
    print("  ego dest  error: %.3fm" % d_dest)
    errors = [d_spawn, d_dest]

    real_npcs = real.get("NPC", [])
    for i, real_npc in enumerate(real_npcs):
        gen_npc = objects_json["NPC"][i]
        real_init = real_npc.get("init_location") or real_npc.get("init_spawn_point")
        d_init = _dist(real_init, gen_npc["init_spawn_point"])
        d_dest_npc = _dist(real_npc["dest_location"], gen_npc["dest_location"])
        print("  NPC[%d] init error: %.3fm, dest error: %.3fm" % (i, d_init, d_dest_npc))
        errors += [d_init, d_dest_npc]
    return max(errors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenarios", nargs="*")
    parser.add_argument("--all", action="store_true", help="verify every converted data/*/LawXX_basic_info.json")
    parser.add_argument("--threshold", type=float, default=2.0, help="flag scenarios with max error above this (m)")
    args = parser.parse_args()

    if args.all:
        # exclude data/<town>/<town>_basic_info.json (gen_town01_routes.py's own
        # default seed, not a traffic_rule_BehAVExplor conversion)
        names = sorted(
            os.path.basename(p)[: -len("_basic_info.json")]
            for p in glob.glob(os.path.join(REPO_ROOT, "data", "*", "*_basic_info.json"))
            if os.path.basename(p)[: -len("_basic_info.json")] != os.path.basename(os.path.dirname(p))
        )
    else:
        names = args.scenarios

    flagged = []
    for name in names:
        try:
            max_err = verify(name)
            if max_err > args.threshold:
                flagged.append((name, max_err))
        except Exception as e:
            print("=== %s ===\n  ERROR: %s" % (name, e))
            flagged.append((name, None))

    print("\nchecked %d scenarios" % len(names))
    if flagged:
        print("flagged (> %.1fm or errored):" % args.threshold)
        for name, err in flagged:
            print("  %s: %s" % (name, "%.3fm" % err if err is not None else "ERROR"))


if __name__ == "__main__":
    main()
