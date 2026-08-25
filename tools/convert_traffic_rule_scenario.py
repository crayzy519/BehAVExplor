"""
Convert traffic_rule_BehAVExplor/LawXX/objects.json (real, human-authored
ego + NPC world-frame spawn/dest points) into a BehAVExplor basic_info.json
seed: world coordinates are matched to the nearest lane_id/offset in
data/<map>/<map>_routes.json (built by gen_town01_routes.py), and NPC
routes that span more than one lane are connected via BFS over the lane
successor graph.

Only fixes up positions - it does not invent new scenario semantics: ego
gets a plain lane_id/offset pair, NPCs (if any) get a "fixed_npc" block
that main.py feeds into Scenario.set_fixed_npc_info() for the first init
seed instead of random NPC generation.

Handles every town that has a data/<town>/<town>_routes.json (built by
tools/gen_town01_routes.py --town <TownXX>); the scenario's own town comes
from its settings.yaml. Scenarios with no successor chain, a pedestrian
NPC, or unreadable objects.json are reported and skipped rather than
aborting the whole batch.

Usage: python3 tools/convert_traffic_rule_scenario.py Law1 Law2 Law27
       python3 tools/convert_traffic_rule_scenario.py --all
"""
import argparse
import json
import math
import os
import re
from collections import deque

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(REPO_ROOT, "traffic_rule_BehAVExplor")

EGO_AGENT_TYPE = "2e966a70-4a19-44b5-a5e7-64e00a7bc5de"
DEFAULT_ENVIRONMENT = {"rain": 0, "fog": 0, "wetness": 0, "cloudiness": 0, "damage": 0, "time": 0}


def _scenario_town(law_dir):
    text = open(os.path.join(law_dir, "settings.yaml")).read()
    m = re.search(r"town:\s*'([^']+)'", text)
    if not m:
        raise RuntimeError("no town: in settings.yaml")
    return m.group(1)


_ROUTES_CACHE = {}


def _load_lanes(town):
    if town not in _ROUTES_CACHE:
        town_slug = town.lower()
        path = os.path.join(REPO_ROOT, "data", town_slug, "%s_routes.json" % town_slug)
        _ROUTES_CACHE[town] = json.load(open(path))["lane_details"]
    return _ROUTES_CACHE[town]


def _polyline(lane):
    return [(p["x"], p["y"]) for p in lane["central"]["points"]]


def _cumulative_lengths(xy):
    cum = [0.0]
    for i in range(1, len(xy)):
        cum.append(cum[-1] + math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1]))
    return cum


def _point_at_offset(xy, cum, offset):
    total = cum[-1]
    offset = max(0.0, min(offset, total))
    for i in range(1, len(cum)):
        if cum[i] >= offset:
            seg_len = cum[i] - cum[i - 1]
            t = 0.0 if seg_len == 0 else (offset - cum[i - 1]) / seg_len
            x0, y0 = xy[i - 1]
            x1, y1 = xy[i]
            return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
    return xy[-1]


def _dist_point_seg(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    denom = dx * dx + dy * dy
    t = 0.0 if denom == 0 else ((px - x0) * dx + (py - y0) * dy) / denom
    t = max(0.0, min(1.0, t))
    cx, cy = x0 + t * dx, y0 + t * dy
    return math.hypot(px - cx, py - cy), t


def nearest_lane_point(lanes, px, py):
    """(x, y) -> (lane_id, offset_along_lane, distance). Point-to-segment search
    over every CITY_DRIVING lane's centerline (lanes is already filtered to
    CITY_DRIVING by gen_town01_routes.py::build_lane_details)."""
    best = (float("inf"), None, None)
    for lane_id, lane in lanes.items():
        xy = _polyline(lane)
        cum = _cumulative_lengths(xy)
        for i in range(1, len(xy)):
            x0, y0 = xy[i - 1]
            x1, y1 = xy[i]
            d, t = _dist_point_seg(px, py, x0, y0, x1, y1)
            if d < best[0]:
                offset = cum[i - 1] + t * (cum[i] - cum[i - 1])
                best = (d, lane_id, offset)
    return best[1], best[2], best[0]


def _bfs_chain(lanes, start_lane_id, end_lane_id, edge_fn, max_hops):
    """edge_fn(lane_dict) -> [(neighbor_id, kind), ...]. Returns (chain, kinds)
    where kinds[i] describes the hop from chain[i] to chain[i+1]."""
    if start_lane_id == end_lane_id:
        return [start_lane_id], []
    q = deque([([start_lane_id], [])])
    seen = {start_lane_id}
    while q:
        chain, kinds = q.popleft()
        if len(chain) > max_hops:
            continue
        for sid, kind in edge_fn(lanes[chain[-1]]):
            if sid not in lanes:
                continue
            if sid == end_lane_id:
                return chain + [sid], kinds + [kind]
            if sid not in seen:
                seen.add(sid)
                q.append((chain + [sid], kinds + [kind]))
    return None, None


def _successor_edges(l):
    return [(e["id"], "succ") for e in l["successor"]]


def _all_edges(l):
    return _successor_edges(l) + [(e["id"], "nbr") for e in l["left_neighbor_lane"] + l["right_neighbor_lane"]]


def lane_chain_between(lanes, start_lane_id, end_lane_id, max_hops=15):
    """BFS over successor AND left/right-neighbor links - real routes often
    need a lane change (e.g. start/dest differ only by lane index on the
    same road), which a successor-only search can never reach since it's a
    lateral, not sequential, relationship. Only safe for ego: its chain is
    recomputed fresh every fuzzing iteration by objects_writer.py, which
    correctly reads arc-length off the concatenated polyline regardless of
    hop type."""
    chain, _ = _bfs_chain(lanes, start_lane_id, end_lane_id, _all_edges, max_hops)
    return chain


def lane_chain_with_kinds_between(lanes, start_lane_id, end_lane_id, max_hops=15):
    """Same search as lane_chain_between, but also returns each hop's kind
    (successor vs neighbor) - needed by build_virtual_lane to know how to
    stitch each lane's points onto the previous one."""
    return _bfs_chain(lanes, start_lane_id, end_lane_id, _all_edges, max_hops)


def successor_chain_between(lanes, start_lane_id, end_lane_id, max_hops=15):
    """Successor-only BFS, for NPC routes. NPC routes get baked into
    npc_routes[i] and re-mutated every generation by Scenario._uniform_mutation/
    _gauss_mutation, which always adds a lane's FULL length when moving to the
    next lane in the chain - correct for a successor hop, silently wrong for a
    neighbor (lane-change) hop. So NPC routes can't use lane_chain_between."""
    chain, _ = _bfs_chain(lanes, start_lane_id, end_lane_id, _successor_edges, max_hops)
    return chain


def _nearest_point_on_polyline(xy, cum, px, py):
    best = (float("inf"), None)
    for i in range(1, len(xy)):
        x0, y0 = xy[i - 1]
        x1, y1 = xy[i]
        d, t = _dist_point_seg(px, py, x0, y0, x1, y1)
        if d < best[0]:
            best = (d, cum[i - 1] + t * (cum[i] - cum[i - 1]))
    return best[1], best[0]


def build_virtual_lane(lanes, chain, kinds, init_xy, dest_xy):
    """Flatten a multi-lane chain (successor and/or left/right-neighbor hops),
    trimmed to the NPC's real start/end world points, into ONE synthetic
    lane_details entry.

    A route that needs a lane change can't be stored as a multi-lane
    npc_routes[i] chain (see successor_chain_between's docstring - Scenario's
    native mutation always adds a lane's FULL length per hop, wrong for a
    neighbor hop). Flattening it into a single lane sidesteps that: the
    synthetic lane's own length is the true point-to-point arc length of the
    trimmed, concatenated polyline (correct regardless of hop type), and
    downstream code (mutation, objects_writer) just sees an ordinary
    single-lane route - no hop-type bookkeeping needed anywhere.

    A successor hop naturally continues (lane B's own points start where
    lane A ends), so we ride the outgoing lane out to its own full end
    before switching. A neighbor hop does NOT continue - neighbor lanes run
    parallel, covering roughly the same real-world span at roughly the same
    arc-length parameterization (validated: every neighbor pair seen so far
    has matching start/end points within a few meters) - so a lane change
    is modeled as an immediate lateral hop at the SAME offset value,
    switching which lane's points get read from there on, rather than
    riding the current lane out to its own end first.

    Trade-off: this bakes the lane-change point into fixed geometry: later
    mutation can move the NPC's progress/speed along this fixed path, but
    can never relocate where the lane change itself happens.
    """
    def lane_xy_cum(lane_id):
        xy = _polyline(lanes[lane_id])
        return xy, _cumulative_lengths(xy)

    start_xy, start_cum = lane_xy_cum(chain[0])
    start_off, _ = _nearest_point_on_polyline(start_xy, start_cum, *init_xy)
    end_xy, end_cum = lane_xy_cum(chain[-1])
    end_off, _ = _nearest_point_on_polyline(end_xy, end_cum, *dest_xy)

    points = []
    reference_offset = start_off
    for k, lane_id in enumerate(chain):
        lane_xy, lane_cum = lane_xy_cum(lane_id)
        if k == len(chain) - 1:
            local_end = end_off
        elif kinds[k] == "succ":
            local_end = lane_cum[-1]
        else:
            local_end = reference_offset  # hand off immediately, no forward travel on this lane

        points.append(_point_at_offset(lane_xy, lane_cum, reference_offset))
        for i, c in enumerate(lane_cum):
            if reference_offset < c < local_end:
                points.append(lane_xy[i])
        points.append(_point_at_offset(lane_xy, lane_cum, local_end))

        if k < len(chain) - 1:
            reference_offset = 0.0 if kinds[k] == "succ" else reference_offset

    length = _cumulative_lengths(points)[-1]
    speed_limit = lanes[chain[0]]["speed_limit"]
    lane_entry = {
        "id": None,  # filled in by the caller once it picks the synthetic id
        "turn": "NO_TURN",
        "speed_limit": speed_limit,
        "right_neighbor_lane": [],
        "left_neighbor_lane": [],
        "predecessor": [],
        "successor": [],
        "central": {
            "points": [{"x": x, "y": y} for x, y in points],
            "length": length,
        },
        "left_boundary": {"type": ["CURB"], "points": []},
        "right_boundary": {"type": ["CURB"], "points": []},
    }
    return lane_entry, length


def convert_ego(lanes, objects_json):
    ego_cfg = next(o for o in objects_json["ego"] if "spawn_point" in o)
    sp, dp = ego_cfg["spawn_point"], ego_cfg["dest_point"]
    start_lane, start_off, start_d = nearest_lane_point(lanes, sp["x"], sp["y"])
    dest_lane, dest_off, dest_d = nearest_lane_point(lanes, dp["x"], dp["y"])
    print("  ego start -> %s @ %.2fm (match dist %.3fm)" % (start_lane, start_off, start_d))
    print("  ego dest  -> %s @ %.2fm (match dist %.3fm)" % (dest_lane, dest_off, dest_d))
    if lane_chain_between(lanes, start_lane, dest_lane) is None:
        raise RuntimeError("No successor chain from ego start %s to dest %s" % (start_lane, dest_lane))
    return {
        "agent_type": EGO_AGENT_TYPE,
        "start": {"lane_id": start_lane, "offset": round(start_off, 2)},
        "destination": {"lane_id": dest_lane, "offset": round(dest_off, 2)},
    }


def convert_npc(lanes, npc_cfg, idx):
    init = npc_cfg.get("init_location") or npc_cfg.get("init_spawn_point")
    dest = npc_cfg["dest_location"]
    speed = float(npc_cfg.get("speed", npc_cfg.get("target_speed", 1.0)))

    start_lane, start_off, start_d = nearest_lane_point(lanes, init["x"], init["y"])
    end_lane, end_off, end_d = nearest_lane_point(lanes, dest["x"], dest["y"])
    print("  NPC[%d] %s: %s @ %.2fm (%.3fm) -> %s @ %.2fm (%.3fm), speed=%.2f"
          % (idx, npc_cfg.get("type"), start_lane, start_off, start_d, end_lane, end_off, end_d, speed))

    chain = successor_chain_between(lanes, start_lane, end_lane)
    if chain is not None:
        # 2 (offset, speed) points per lane in the chain (basic_info's npcs.waypoint
        # must be set to 2 to match) - a single-lane chain still needs 2 slots to
        # carry both the real start point and the real destination point.
        n = len(chain)
        offset_speed = []
        running_length = 0.0
        for k, lane_id in enumerate(chain):
            lane_length = lanes[lane_id]["central"]["length"]
            if k == 0 and k == n - 1:
                pts = [(start_off, speed), (end_off, 0.0)]
            elif k == 0:
                pts = [(start_off, speed), (lane_length, speed)]
            elif k == n - 1:
                pts = [(0.0, speed), (end_off, 0.0)]
            else:
                pts = [(lane_length / 3.0, speed), (2 * lane_length / 3.0, speed)]
            for local_offset, wp_speed in pts:
                offset_speed.append([round(running_length + local_offset, 2), round(wp_speed, 3)])
            running_length += lane_length
        return chain, offset_speed, {}

    # No pure-successor path - needs a lane change. Flatten the full
    # (successor + neighbor) chain into one synthetic lane instead.
    full_chain, kinds = lane_chain_with_kinds_between(lanes, start_lane, end_lane)
    if full_chain is None:
        raise RuntimeError("NPC[%d]: no successor/neighbor chain from %s to %s"
                            % (idx, start_lane, end_lane))
    lane_entry, length = build_virtual_lane(lanes, full_chain, kinds, (init["x"], init["y"]), (dest["x"], dest["y"]))
    synthetic_id = "npc%d_lanechange_%s_to_%s" % (idx, start_lane, end_lane)
    lane_entry["id"] = synthetic_id
    print("  NPC[%d]: needs a lane change (%s) - flattened into synthetic lane %s (%.2fm)"
          % (idx, " -> ".join(full_chain), synthetic_id, length))
    offset_speed = [[0.0, round(speed, 3)], [round(length, 2), 0.0]]
    return [synthetic_id], offset_speed, {synthetic_id: lane_entry}


def convert_scenario(law_name):
    src_dir = os.path.join(SRC_DIR, law_name)
    print("=== %s ===" % law_name)

    town = _scenario_town(src_dir)
    town_slug = town.lower()
    lanes = _load_lanes(town)

    objects_path = os.path.join(src_dir, "objects.json")
    if os.path.getsize(objects_path) == 0:
        raise RuntimeError("objects.json is empty (0 bytes) - broken source data")
    objects_json = json.load(open(objects_path))

    ego = convert_ego(lanes, objects_json)

    npc_list = objects_json.get("NPC", [])
    npc_types, npc_routes, npc_waypoints = [], [], []
    synthetic_lanes = {}
    for i, npc_cfg in enumerate(npc_list):
        if "walker" in npc_cfg.get("type", ""):
            raise RuntimeError("NPC[%d] is a pedestrian - BehAVExplor's mutation algorithm "
                                "has no pedestrian model" % i)
        chain, offset_speed, npc_synthetic_lanes = convert_npc(lanes, npc_cfg, i)
        npc_types.append("Sedan")  # blueprint identity isn't preserved by NPC_AGENT_TYPES; closest generic car
        npc_routes.append(chain)
        npc_waypoints.append(offset_speed)
        synthetic_lanes.update(npc_synthetic_lanes)

    basic_info = {
        "name": law_name,
        "map": town,
        "settings_yaml": os.path.relpath(os.path.join(src_dir, "settings.yaml"), REPO_ROOT),
        "ego": ego,
        "npcs": {"waypoint": 2, "npc_num": len(npc_list)},
        "environment": dict(DEFAULT_ENVIRONMENT),
    }
    if npc_list:
        basic_info["fixed_npc"] = {"types": npc_types, "routes": npc_routes, "waypoints": npc_waypoints}
    if synthetic_lanes:
        basic_info["synthetic_lanes"] = synthetic_lanes

    out_dir = os.path.join(REPO_ROOT, "data", town_slug)
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "%s_basic_info.json" % law_name)
    with open(out_path, "w") as f:
        json.dump([basic_info], f, indent=2)
    print("  wrote %s" % os.path.relpath(out_path, REPO_ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenarios", nargs="*", help="LawXX folder names under traffic_rule_BehAVExplor/")
    parser.add_argument("--all", action="store_true", help="convert every LawXX folder under traffic_rule_BehAVExplor/")
    args = parser.parse_args()

    names = sorted(os.listdir(SRC_DIR)) if args.all else args.scenarios

    ok, failed = [], []
    for name in names:
        if not os.path.isdir(os.path.join(SRC_DIR, name)):
            continue
        try:
            convert_scenario(name)
            ok.append(name)
        except Exception as e:
            print("  SKIP: %s" % e)
            failed.append((name, str(e)))

    print("\n%d converted, %d skipped" % (len(ok), len(failed)))
    if failed:
        print("skipped: " + ", ".join("%s (%s)" % (n, e) for n, e in failed))


if __name__ == "__main__":
    main()
