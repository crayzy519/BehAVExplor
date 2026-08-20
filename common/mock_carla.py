"""
MockSimulator: run BehAVExplor's fuzzing loop against a MOCKED CARLA world.

This is a drop-in replacement for common.simulator.Simulator (same constructor +
run()/close() contract that common.runner.Runner calls), used while no real CARLA
is running.

Layering (important)
--------------------
This class does NOT fabricate LGSVL-shaped frames directly. Instead it mirrors
what the real CARLA simulator will do:

    1. build a world of actors            -> MockCarlaWorld / MockCarlaActor
                                              (common/mock_carla_world.py)
    2. step the world                     -> world.tick()
    3. read each actor's state            -> carla_adapter.state_from_carla(actor)
                                              (common/carla_adapter.py, REAL code)
    4. feed FrameElement / CaseRecorder   -> common/frame.py (unchanged)

Only step 1-2's *producer* is mocked. Steps 3-4 are the real migration path, so
switching to live CARLA later means replacing MockCarlaWorld with
client.get_world() and MockCarlaActor with spawned carla vehicles -- the adapter
and everything downstream stay identical.

Trajectory synthesis
---------------------
We still need scripted motion (no physics engine here), so we synthesise a
per-actor trajectory in CARLA coordinates: the ego cruises along its start-lane
centerline from start offset to destination offset; NPCs are currently parked
(see _npc_trajectory). map.json points are (x, y) world coords -> CARLA
Location(x, -y) because the adapter later flips Y back (bridge convention).
"""
import math
from loguru import logger

from common.frame import CaseRecorder, FrameElement
from common.mock_carla_world import (
    MockCarlaWorld, MockCarlaActor, Transform, Location, Rotation, Vector3D,
)
from common import carla_adapter


# --------------------------------------------------------------------------- #
# Centerline geometry helpers (arc-length parametrization), CARLA frame
# --------------------------------------------------------------------------- #
def _polyline(points):
    """map.json central points [{x,y}] -> CARLA-frame (x, y): y negated."""
    # map (x, y) world coords; CARLA Location uses y = -world_y (bridge flips back)
    return [(p["x"], -p["y"]) for p in points]


def _cumulative_lengths(xy):
    lengths = [0.0]
    for i in range(1, len(xy)):
        dx = xy[i][0] - xy[i - 1][0]
        dy = xy[i][1] - xy[i - 1][1]
        lengths.append(lengths[-1] + math.hypot(dx, dy))
    return lengths


def _point_at_offset(xy, cum, offset):
    """Interpolate (x, y, yaw_deg) at arc-length `offset` along the CARLA polyline."""
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


def _lines_to_xz(lines):
    """[[{x,y}, ...]] -> [[(x, z), ...]] for shapely; z = -map.y (adapter frame)."""
    return [[(p["x"], -p["y"]) for p in line] for line in lines]


# --------------------------------------------------------------------------- #
# Trajectory builders (produce CARLA-frame frames for MockCarlaActor)
# --------------------------------------------------------------------------- #
def _frame(x, y, yaw, vx=0.0, vy=0.0):
    return {
        "transform": Transform(Location(x, y, 0.0), Rotation(yaw=yaw)),
        "velocity": Vector3D(vx, vy, 0.0),
        "ang_vel": Vector3D(),
        "accel": Vector3D(),
    }


def _ego_trajectory(lgsvl_input, dt):
    ego = lgsvl_input["ego"]
    lanes = lgsvl_input["lanes"]
    start_lane = ego["start"]["lane_id"]
    start_off = float(ego["start"]["offset"])
    dest_off = float(ego["destination"]["offset"])

    xy = _polyline(lanes[start_lane]["central"]["points"])
    cum = _cumulative_lengths(xy)

    speed = min(float(lanes[start_lane]["speed_limit"]), 8.0)
    distance = abs(dest_off - start_off)
    direction = 1.0 if dest_off >= start_off else -1.0
    n_steps = max(2, int(distance / (speed * dt)) + 1)

    frames = []
    for k in range(n_steps):
        off = start_off + direction * speed * dt * k
        x, y, yaw = _point_at_offset(xy, cum, off)
        # velocity from next sample
        off2 = start_off + direction * speed * dt * (k + 1)
        x2, y2, _ = _point_at_offset(xy, cum, off2)
        frames.append(_frame(x, y, yaw, (x2 - x) / dt, (y2 - y) / dt))
    # final stopped frame at destination
    x, y, yaw = _point_at_offset(xy, cum, dest_off)
    frames.append(_frame(x, y, yaw, 0.0, 0.0))
    return frames


_NPC_SPEED_FLOOR = 0.2  # matches scenario.min_speed_limit; avoids stalling


def _npc_trajectory(lgsvl_input, npc_id, dt, max_frames):
    """Follow the (mutated) waypoints: [offset, speed] along the route centerline.

    offset is the cumulative arc-length from the route start (see
    scenario._uniform_mutation), so we concatenate every route lane's centerline
    into one polyline and place the NPC at each waypoint's offset, cruising to the
    next at that segment's speed. Truncated to max_frames so the sim terminates.
    """
    lanes = lgsvl_input["lanes"]
    routes = lgsvl_input["npc_lanes_to_follow"]
    all_waypoints = lgsvl_input["npc_waypoints"]
    route = routes[npc_id] if npc_id < len(routes) else None
    waypoints = all_waypoints[npc_id] if npc_id < len(all_waypoints) else None
    if not route or not waypoints:
        return [_frame(9999.0, 9999.0, 0.0)]

    # concatenate all route lanes' centerlines into one arc-length polyline
    xy = []
    for lane_id in route:
        xy.extend(_polyline(lanes[lane_id]["central"]["points"]))
    cum = _cumulative_lengths(xy)

    # walk offset from waypoint to waypoint at that segment's cruising speed
    offsets = [float(waypoints[0][0])]
    for w in range(1, len(waypoints)):
        target = float(waypoints[w][0])
        seg_speed = max(float(waypoints[w - 1][1]), _NPC_SPEED_FLOOR)
        cur = offsets[-1]
        while cur < target - 1e-6 and len(offsets) < max_frames:
            cur = min(cur + seg_speed * dt, target)
            offsets.append(cur)
        if len(offsets) >= max_frames:
            break

    # sample positions; velocity from consecutive offsets
    frames = []
    for k, off in enumerate(offsets):
        x, y, yaw = _point_at_offset(xy, cum, off)
        if k + 1 < len(offsets):
            x2, y2, _ = _point_at_offset(xy, cum, offsets[k + 1])
            frames.append(_frame(x, y, yaw, (x2 - x) / dt, (y2 - y) / dt))
        else:
            frames.append(_frame(x, y, yaw, 0.0, 0.0))
    return frames


# --------------------------------------------------------------------------- #
# Mock simulator
# --------------------------------------------------------------------------- #
class MockSimulator(object):

    def __init__(self, max_sim_time, lgsvl_map=None, apollo_map=None, sim_mode="mock", dt=0.1):
        self.max_sim_time = max_sim_time
        self.dt = dt
        logger.info("[MockSimulator] init: mocked CARLA world. map=%s mode=%s"
                    % (lgsvl_map, sim_mode))

    def run(self, scenario_obj, scenario_id, record_apollo_path=None):
        lgsvl_input = scenario_obj.get_lgsvl_input()

        # 1. build the (mock) CARLA world + actors -------------------------- #
        world = MockCarlaWorld(dt=self.dt)
        ego_frames = _ego_trajectory(lgsvl_input, self.dt)
        ego_actor = world.add_actor(MockCarlaActor(ego_frames))

        # cap NPC frames so a slow NPC can't run the sim forever
        max_frames = int(self.max_sim_time / self.dt) + 1
        npc_size = lgsvl_input["npc_size"]
        npc_actors = [
            world.add_actor(MockCarlaActor(_npc_trajectory(lgsvl_input, i, self.dt, max_frames)))
            for i in range(npc_size)
        ]

        # collision signal seam: mirror LGSVL's ego.on_collision / CARLA's
        # sensor.other.collision. The mock sensor never fires (no physics), so
        # collision_flag stays False -- swap in a real spawned sensor later and
        # neither this callback nor the add_frame() read below changes.
        collision_flag = {"hit": False}
        collision_sensor = world.spawn_collision_sensor(ego_actor)
        collision_sensor.listen(lambda event: collision_flag.__setitem__("hit", True))

        # destination (last ego frame) -> LGSVL-shaped dest via adapter
        dest_loc = ego_frames[-1]["transform"].location
        destination = carla_adapter.location_to_dest(dest_loc)

        case_recorder = CaseRecorder(scenario_id)
        case_recorder.set_destination(destination)

        yellow = _lines_to_xz(lgsvl_input["yellow_lines"])
        edge = _lines_to_xz(lgsvl_input["edge_lines"])
        cross = _lines_to_xz(lgsvl_input["cross_lines"])

        # bboxes read once through the adapter (shape is constant)
        ego_bbox = carla_adapter.bbox_from_carla(ego_actor)
        npc_bboxes = [carla_adapter.bbox_from_carla(a) for a in npc_actors]

        # 2. step the world + 3. read state via adapter + 4. feed frame.py -- #
        frame_id = 0
        while True:
            sim_time = world.get_snapshot().timestamp.elapsed_seconds

            ego_state = carla_adapter.state_from_carla(ego_actor)
            frame_npc_info = [
                {"npc_id": i, "npc_bbox": npc_bboxes[i],
                 "npc_state": carla_adapter.state_from_carla(npc_actors[i])}
                for i in range(npc_size)
            ]

            frame_element = FrameElement(
                frame_id, sim_time, destination, ego_bbox, ego_state,
                frame_npc_info, yellow, edge, cross,
            )
            _, end = case_recorder.add_frame(frame_element, collision=collision_flag["hit"])
            if end or world.all_finished():
                break

            world.tick()
            frame_id += 1

        case_recorder.offline_analyze()
        logger.info("[MockSimulator] %s: %d frames, events=[%s]"
                    % (scenario_id, len(case_recorder.frames),
                       case_recorder.obtain_case_event_str()))
        return case_recorder

    def close(self):
        logger.info("[MockSimulator] closed.")
