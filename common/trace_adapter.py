"""
trace.json -> BehAVExplor (LGSVL-shaped) state adapter.

trace.json is what the real CARLA bridge produces (see
carla_bridge/scenario_executor.py::printTrace), one entry per simulation
frame: {"TimeStamp", "Sequence", "EGO": {...}, "NPCs": [...], "Traffic_Lights": []}.
Each actor dict carries Position/Rotation/Scale/AngularVelocity/Velocity/
LinearVelocity/Acceleration/ForwardRight, in the bridge's own frame:
    Position  = {x: carla.x,        y: -carla.y,        z: carla.z}
    Rotation  = {x: carla.roll,     y: carla.pitch,      z: -carla.yaw}
    Scale     = full bounding-box extents (2x carla half-extents)
    Velocity  = {x: carla.vx,       y: -carla.vy,        z: carla.vz}
"""


class _V3(object):
    __slots__ = ("x", "y", "z")

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)


class _Transform(object):
    __slots__ = ("position", "rotation")

    def __init__(self, position, rotation):
        self.position = position
        self.rotation = rotation


class LGSVLState(object):
    __slots__ = ("transform", "velocity", "angular_velocity")

    def __init__(self, transform, velocity, angular_velocity):
        self.transform = transform
        self.velocity = velocity
        self.angular_velocity = angular_velocity


class LGSVLBBox(object):
    __slots__ = ("min", "max")

    def __init__(self, min_v, max_v):
        self.min = min_v
        self.max = max_v


def state_from_trace(actor):
    """trace.json EGO/NPC entry -> LGSVLState."""
    pos, rot = actor["Position"], actor["Rotation"]
    vel, ang = actor["Velocity"], actor["AngularVelocity"]

    position = _V3(pos["x"], pos["z"], pos["y"])
    rotation = _V3(0.0, rot["z"], 0.0)
    transform = _Transform(position, rotation)

    velocity = _V3(vel["x"], 0.0, vel["y"])
    angular_velocity = _V3(ang["x"], ang["z"], ang["y"])

    return LGSVLState(transform, velocity, angular_velocity)


def bbox_from_trace(actor):
    """trace.json Scale (full extents) -> LGSVLBBox (min/max, vehicle-local)."""
    scale = actor["Scale"]
    half_length = scale["x"] / 2.0
    half_width = scale["y"] / 2.0
    height = scale["z"]
    min_v = _V3(-half_length, 0.0, -half_width)
    max_v = _V3(half_length, height, half_width)
    return LGSVLBBox(min_v, max_v)


def world_point_to_destination(x, y):
    """world-frame (x, y) [objects.json ego dest_point] -> CaseRecorder destination (.x/.z)."""
    class _Dest(object):
        __slots__ = ("x", "z")
    d = _Dest()
    d.x = x
    d.z = y
    return d
