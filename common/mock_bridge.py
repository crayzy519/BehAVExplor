"""
Mock CARLA bridge: consumes an objects.json (world-frame spawn/dest/waypoints,
see common/objects_writer.py) and writes a trace.json into case_dir, shaped
exactly like carla_bridge/scenario_executor.py::printTrace()'s output (see
trace.json.txt for a real-bridge sample). This mirrors the real bridge's
file-based contract: caller writes objects.json to case_dir, bridge reads it
and writes trace.json back to the same directory.

适配真实carla bridge时可以移除。
"""
import json
import math
import os

_DEFAULT_SCALE = {"x": 4.5, "y": 2.0, "z": 1.5}
_SPEED_FLOOR = 0.2


def _yaw_deg(x0, y0, x1, y1):
    if x1 == x0 and y1 == y0:
        return 0.0
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


def _actor_frame(role_id, x, y, z, yaw_deg, vx, vy, label=None):
    entry = {
        "Id": role_id,
        "Position": {"x": x, "y": y, "z": z},
        "Rotation": {"x": 0.0, "y": 0.0, "z": yaw_deg},
        "Scale": dict(_DEFAULT_SCALE),
        "AngularVelocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "Velocity": {"x": vx, "y": vy, "z": 0.0},
        "LinearVelocity": {"x": math.hypot(vx, vy), "y": 0.0, "z": 0.0},
        "Acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
        "ForwardRight": {"x": 0.0, "y": 0.0, "z": 0.0},
    }
    if label:
        entry["Label"] = label
    return entry


def _sample_segments(points, dt, max_frames):
    """points: [(x, y, z, speed_to_reach_this_point), ...] -> resampled (x, y, z, yaw_deg) at dt.

    speed_to_reach_this_point (points[i][3]) is the target cruising speed for
    the segment ENDING at points[i] - mirrors carla_bridge/utils/load_NPCs.py's
    run() (front_wp's own speed drives the segment approaching it).

    yaw is carried from the segment each sample was generated on, so the final
    (stationary) sample keeps the heading of the last real segment instead of
    collapsing to 0 once consecutive samples coincide.
    """
    samples = [points[0][:3] + (0.0,)]
    for i in range(1, len(points)):
        x0, y0, z0, _ = points[i - 1]
        x1, y1, z1, speed = points[i]
        speed = max(float(speed), _SPEED_FLOOR)
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len <= 1e-6:
            continue
        yaw = _yaw_deg(x0, y0, x1, y1)
        samples[-1] = (samples[-1][0], samples[-1][1], samples[-1][2], yaw)
        n_steps = max(1, int(seg_len / (speed * dt)))
        for step in range(1, n_steps + 1):
            t = min(1.0, step / n_steps)
            samples.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0), z0 + t * (z1 - z0), yaw))
            if len(samples) >= max_frames:
                return samples
    return samples


def run(objects_json, max_sim_time, case_dir, dt=0.1):
    """objects.json dict -> writes trace.json (frame-dict list) into case_dir."""
    max_frames = int(max_sim_time / dt) + 1

    ego_cfg = next(o for o in objects_json["ego"] if "spawn_point" in o)
    sp, dp = ego_cfg["spawn_point"], ego_cfg["dest_point"]
    ego_speed = max(float(ego_cfg.get("target_speed", 5.0)), _SPEED_FLOOR)
    ego_points = [
        (sp["x"], sp["y"], sp["z"], ego_speed),
        (dp["x"], dp["y"], dp["z"], ego_speed),
    ]
    ego_samples = _sample_segments(ego_points, dt, max_frames)

    npc_samples = []
    for npc in objects_json["NPC"]:
        wps = npc["waypoints"]
        points = [(w["x"], w["y"], w["z"], w["speed"]) for w in wps]
        npc_samples.append((npc["role_name"], _sample_segments(points, dt, max_frames)))

    n_frames = min(max_frames, max([len(ego_samples)] + [len(s) for _, s in npc_samples] + [2]))

    trace = []
    for k in range(n_frames):
        ego_i = min(k, len(ego_samples) - 1)
        ex, ey, ez, eyaw = ego_samples[ego_i]
        ex2, ey2, _, _ = ego_samples[min(ego_i + 1, len(ego_samples) - 1)]
        evx, evy = (ex2 - ex) / dt, (ey2 - ey) / dt

        frame = {
            "TimeStamp": k * dt,
            "Sequence": k,
            "EGO": _actor_frame(ego_cfg["id"], ex, ey, ez, eyaw, evx, evy),
            "NPCs": [],
            "Traffic_Lights": [],
        }

        for role_name, samples in npc_samples:
            i = min(k, len(samples) - 1)
            nx, ny, nz, nyaw = samples[i]
            nx2, ny2, _, _ = samples[min(i + 1, len(samples) - 1)]
            nvx, nvy = (nx2 - nx) / dt, (ny2 - ny) / dt
            frame["NPCs"].append(_actor_frame(role_name, nx, ny, nz, nyaw, nvx, nvy, label="Vehicle"))

        trace.append(frame)

    with open(os.path.join(case_dir, "trace.json"), "w") as f:
        json.dump(trace, f, indent=2)
