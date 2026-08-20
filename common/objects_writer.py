"""
Scenario -> objects.json.

Builds the CARLA-bridge-shaped config the fuzzing loop hands off to the
simulator each iteration: ego spawn/dest points under "ego" (schema mirrors
carla_bridge/config/follow/objects.json) plus a mutated "NPC" list (schema
mirrors carla_bridge/config/follow/NPCs.json), each NPC carrying an ordered
"waypoints" list of {x, y, z, speed}
"""
import math

NPC_BLUEPRINTS = {
    "Sedan": "vehicle.tesla.model3",
    "SUV": "vehicle.nissan.patrol",
    "Jeep": "vehicle.jeep.wrangler_rubicon",
    "Hatchback": "vehicle.audi.a2",
    "SchoolBus": "vehicle.volkswagen.t2",
}
EGO_BLUEPRINT = "vehicle.lincoln.mkz_2017"
DEFAULT_Z = 0.3


def _polyline(points):
    return [(p["x"], p["y"]) for p in points]


def _cumulative_lengths(xy):
    lengths = [0.0]
    for i in range(1, len(xy)):
        dx = xy[i][0] - xy[i - 1][0]
        dy = xy[i][1] - xy[i - 1][1]
        lengths.append(lengths[-1] + math.hypot(dx, dy))
    return lengths


def _point_at_offset(xy, cum, offset):
    """Interpolate (x, y, yaw_deg) at arc-length `offset` along a world polyline."""
    total = cum[-1]
    offset = max(0.0, min(offset, total))
    for i in range(1, len(cum)):
        if cum[i] >= offset:
            seg_len = cum[i] - cum[i - 1]
            t = 0.0 if seg_len == 0 else (offset - cum[i - 1]) / seg_len
            x0, y0 = xy[i - 1]
            x1, y1 = xy[i]
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            yaw = math.degrees(math.atan2(y1 - y0, x1 - x0))
            return x, y, yaw
    x, y = xy[-1]
    return x, y, 0.0


def _ego_geometry(lgsvl_input):
    ego = lgsvl_input["ego"]
    lanes = lgsvl_input["lanes"]
    lane_id = ego["start"]["lane_id"]
    xy = _polyline(lanes[lane_id]["central"]["points"])
    cum = _cumulative_lengths(xy)

    start = _point_at_offset(xy, cum, float(ego["start"]["offset"]))
    dest = _point_at_offset(xy, cum, float(ego["destination"]["offset"]))

    speed_limit = float(lanes[lane_id]["speed_limit"])
    target_speed = min(speed_limit, 8.0) if speed_limit > 0 else 8.0
    return start, dest, target_speed


def _npc_waypoints(lgsvl_input, npc_index):
    """[[offset, speed], ...] (arc-length along the route) -> [{x,y,z,speed}, ...]."""
    lanes = lgsvl_input["lanes"]
    route = lgsvl_input["npc_lanes_to_follow"][npc_index]
    waypoints = lgsvl_input["npc_waypoints"][npc_index]

    xy = []
    for lane_id in route:
        xy.extend(_polyline(lanes[lane_id]["central"]["points"]))
    cum = _cumulative_lengths(xy)

    points = []
    for offset, speed in waypoints:
        x, y, yaw = _point_at_offset(xy, cum, float(offset))
        points.append({"x": x, "y": y, "z": DEFAULT_Z, "yaw": yaw, "speed": float(speed)})
    return points


def _day_and_night(environment):
    time_of_day = float(environment.get("time", 12))
    return "Day" if 6.0 <= time_of_day < 18.0 else "Night"


def build_objects_json(lgsvl_input):
    """scenario_obj.get_lgsvl_input() -> objects.json-shaped dict (world-frame)."""
    (ego_x, ego_y, ego_yaw), (dest_x, dest_y, dest_yaw), ego_speed = _ego_geometry(lgsvl_input)

    npc_entries = []
    for i in range(lgsvl_input["npc_size"]):
        points = _npc_waypoints(lgsvl_input, i)
        first, last = points[0], points[-1]
        blueprint = NPC_BLUEPRINTS.get(lgsvl_input["npc_types"][i], EGO_BLUEPRINT)
        npc_entries.append({
            "type": blueprint,
            "role_name": "npc_vehicle_%d" % i,
            "init_spawn_point": {
                "x": first["x"], "y": first["y"], "z": first["z"],
                "roll": 0.0, "pitch": 0.0, "yaw": first["yaw"],
            },
            "dest_location": {"x": last["x"], "y": last["y"], "z": last["z"]},
            "max_throttle": 1,
            "max_brake": 1,
            "max_steer": 0.6,
            "target_speed": first["speed"],
            "is_follow_speed_limits": 0,
            "waypoints": [
                {"x": p["x"], "y": p["y"], "z": p["z"], "speed": p["speed"]}
                for p in points
            ],
        })

    return {
        "DayAndNight": _day_and_night(lgsvl_input["environment"]),
        "NPC": npc_entries,
        "ego": [
            {"type": "sensor.pseudo.traffic_lights", "id": "traffic_lights"},
            {
                "type": EGO_BLUEPRINT,
                "id": "ego_vehicle",
                "vehicle_name": "Lincoln2017MKZ",
                "spawn_point": {
                    "x": ego_x, "y": ego_y, "z": DEFAULT_Z,
                    "roll": 0.0, "pitch": 0.0, "yaw": ego_yaw,
                },
                "target_speed": ego_speed,
                "dest_point": {
                    "x": dest_x, "y": dest_y, "z": DEFAULT_Z,
                    "roll": 0.0, "pitch": 0.0, "yaw": dest_yaw,
                },
                "sensors": [
                    {"type": "sensor.other.collision", "id": "collision_detector"},
                ],
            },
        ],
    }
