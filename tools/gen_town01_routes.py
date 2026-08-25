"""
Generate BehAVExplor routes.json + basic_info.json from carla_bridge's Apollo HD map.

Input : an Apollo HD map JSON (coords already CARLA world coords)
Output: data/<town>/<town>_routes.json       (lane_details / route_details / routes)
        data/<town>/<town>_basic_info.json   (ego start/dest + npc + environment)

Notes
-----
* BehAVExplor's scenario.py only accepts boundary types CURB / DOUBLE_YELLOW /
  DOTTED_WHITE (it raises on anything else). Apollo maps in this repo only
  carry UNKNOWN / DOTTED_YELLOW, so we map:
      DOTTED_YELLOW -> DOUBLE_YELLOW   (center divider -> yellow oracle line)
      UNKNOWN       -> CURB            (road edge      -> edge oracle line)
* Only CITY_DRIVING lanes are exported (mutation samples driving lanes only).
* central.points / boundary.points drop z; keep {x, y}.
* carla_bridge/Oracle/CarlaTest/map_json/*.json are corrupted (every point's
  y is 0) - use carla_bridge/Oracle/CarlaTest/map_json_bak/*.json instead.

Usage: python3 tools/gen_town01_routes.py [--map-json PATH] [--town NAME] [--out-dir DIR]
Defaults to Town01 / carla_bridge/map.json for backward compatibility.
"""
import argparse
import json
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MAP_JSON = os.path.join(REPO_ROOT, "carla_bridge", "map.json")
DEFAULT_TOWN = "Town01"

BOUNDARY_TYPE_MAP = {
    "DOTTED_YELLOW": "DOUBLE_YELLOW",
    "UNKNOWN": "CURB",
    "DOTTED_WHITE": "DOTTED_WHITE",
    "DOUBLE_YELLOW": "DOUBLE_YELLOW",
    "CURB": "CURB",
    "SOLID_YELLOW": "DOUBLE_YELLOW",
    "SOLID_WHITE": "DOTTED_WHITE",
}


def flatten_curve(curve):
    """Apollo curve {segmentList:[{lineSegment:{pointList:[{x,y,z}]}}]} -> [{x,y}]."""
    pts = []
    for seg in curve.get("segmentList", []):
        for p in seg.get("lineSegment", {}).get("pointList", []):
            pts.append({"x": p["x"], "y": p["y"]})
    return pts


def map_boundary(boundary):
    """Apollo boundary -> {type:[<mapped str>], points:[{x,y}]}."""
    raw_types = []
    for bt in boundary.get("boundaryTypeList", []):
        raw_types.extend(bt.get("typesList", []))
    if not raw_types:
        raw_types = ["UNKNOWN"]
    mapped = [BOUNDARY_TYPE_MAP.get(t, "CURB") for t in raw_types]
    points = flatten_curve(boundary.get("curve", {}))
    return {"type": mapped, "points": points}


def build_lane_details(lane_list):
    driving_ids = {
        l["id"]["id"] for l in lane_list if l.get("type") == "CITY_DRIVING"
    }
    lane_details = {}
    for l in lane_list:
        if l.get("type") != "CITY_DRIVING":
            continue
        lid = l["id"]["id"]

        def ids(key):
            return [
                {"id": x["id"]}
                for x in l.get(key, [])
                if x["id"] in driving_ids
            ]

        central_pts = flatten_curve(l.get("centralCurve", {}))
        lane_details[lid] = {
            "id": lid,
            "turn": l.get("turn", "NO_TURN"),
            "speed_limit": l.get("speedLimit", 0.0),
            "right_neighbor_lane": ids("rightNeighborForwardLaneIdList"),
            "left_neighbor_lane": ids("leftNeighborForwardLaneIdList"),
            "predecessor": ids("predecessorIdList"),
            "successor": ids("successorIdList"),
            "central": {
                "points": central_pts,
                "length": l.get("length", 0.0),
            },
            "left_boundary": map_boundary(l.get("leftBoundary", {})),
            "right_boundary": map_boundary(l.get("rightBoundary", {})),
        }
    return lane_details


def build_routes(lane_details, max_hops=3, max_routes=40):
    """Traverse successor chains to produce multi-lane NPC routes."""
    route_details = {}
    routes = []
    # seed from longest lanes first for interesting long routes
    seeds = sorted(
        lane_details.keys(), key=lambda k: -lane_details[k]["central"]["length"]
    )
    seen_chains = set()
    idx = 0
    for start in seeds:
        chain = [start]
        cur = start
        for _ in range(max_hops - 1):
            succ = lane_details[cur]["successor"]
            if not succ:
                break
            cur = succ[0]["id"]
            if cur in chain:  # avoid loops
                break
            chain.append(cur)
        key = tuple(chain)
        if key in seen_chains:
            continue
        seen_chains.add(key)
        rid = "route_%d" % idx
        route_details[rid] = chain
        routes.append(rid)
        idx += 1
        if idx >= max_routes:
            break
    return route_details, routes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-json", default=DEFAULT_MAP_JSON)
    parser.add_argument("--town", default=DEFAULT_TOWN, help="e.g. Town01, Town04, Town05")
    parser.add_argument("--out-dir", default=None, help="defaults to data/<town lowercased>")
    args = parser.parse_args()

    town_slug = args.town.lower()
    out_dir = args.out_dir or os.path.join(REPO_ROOT, "data", town_slug)

    m = json.load(open(args.map_json))
    lane_list = m["laneList"]
    lane_details = build_lane_details(lane_list)
    route_details, routes = build_routes(lane_details)

    os.makedirs(out_dir, exist_ok=True)
    routes_obj = {
        "lane_details": lane_details,
        "route_details": route_details,
        "routes": routes,
    }
    with open(os.path.join(out_dir, "%s_routes.json" % town_slug), "w") as f:
        json.dump(routes_obj, f, indent=1)

    # pick a long straight lane for ego (start == dest lane, offsets inside length)
    ego_lane = max(lane_details, key=lambda k: lane_details[k]["central"]["length"])
    ego_len = lane_details[ego_lane]["central"]["length"]
    basic = [
        {
            "name": "%s_straight" % town_slug,
            "map": args.town,
            "ego": {
                "agent_type": "2e966a70-4a19-44b5-a5e7-64e00a7bc5de",
                "start": {"lane_id": ego_lane, "offset": 10},
                "destination": {"lane_id": ego_lane, "offset": round(ego_len - 10, 1)},
            },
            "npcs": {"waypoint": 4, "npc_num": 6},
            "environment": {
                "rain": 0, "fog": 0, "wetness": 0,
                "cloudiness": 0, "damage": 0, "time": 0,
            },
        }
    ]
    with open(os.path.join(out_dir, "%s_basic_info.json" % town_slug), "w") as f:
        json.dump(basic, f, indent=4)

    print("lanes:", len(lane_details), "| routes:", len(routes))
    print("ego lane:", ego_lane, "len:", round(ego_len, 1))
    print("wrote", out_dir)


if __name__ == "__main__":
    main()
