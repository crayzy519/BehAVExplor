"""
CARLA -> BehAVExplor state adapter.

BehAVExplor's frame.py / utils.py were written for LGSVL and read agent state as:
    state.transform.position.x / .y / .z
    state.transform.rotation.x / .y / .z      (rotation.y = heading in DEGREES)
    state.velocity.x / .y / .z
    state.angular_velocity.x / .y / .z
and bounding boxes as:
    bbox.min.{x,y,z} / bbox.max.{x,y,z}       (vehicle-local)

CARLA exposes different objects (carla.Transform.location/.rotation.yaw,
carla.Actor.get_velocity(), bounding_box.extent). This module converts the
former into the latter. It is the single seam between CARLA and the unchanged
fuzzing core, so BOTH the real CARLA world and the MockCarlaWorld go through it.

Axis mapping (CARLA -> LGSVL/BehAVExplor)
-----------------------------------------
BehAVExplor's oracles operate in the (x, z) ground plane with rotation.y as the
heading. CARLA's ground plane is (x, y) with yaw as the heading, and CARLA is
left-handed. Following the bridge convention
(carla_bridge/actor/traffic_participant.py:78, which negates location.y), we map:

    LGSVL.transform.position.x  <-  carla.location.x
    LGSVL.transform.position.z  <-  -carla.location.y     (flip -> right-handed)
    LGSVL.transform.position.y  <-  carla.location.z       (up axis; unused by oracles)
    LGSVL.transform.rotation.y  <-  -carla.rotation.yaw    (heading, degrees)
    LGSVL.velocity.x            <-  carla.velocity.x
    LGSVL.velocity.z            <-  -carla.velocity.y
    LGSVL.angular_velocity.*    <-  carla.angular_velocity.* (z flipped)

bbox: CARLA gives half-extents; LGSVL wants min/max. length=extent.x*2 (fwd=x),
width=extent.y*2 (lateral=z in LGSVL frame), height=extent.z*2 (up=y).
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
    """LGSVL-shaped agent state consumed by frame.py / utils.py (unchanged)."""

    __slots__ = ("transform", "velocity", "angular_velocity")

    def __init__(self, transform, velocity, angular_velocity):
        self.transform = transform
        self.velocity = velocity
        self.angular_velocity = angular_velocity


class LGSVLBBox(object):
    """LGSVL-shaped bounding box: .min / .max as vehicle-local _V3."""

    __slots__ = ("min", "max")

    def __init__(self, min_v, max_v):
        self.min = min_v
        self.max = max_v


def state_from_carla(carla_actor):
    """carla.Actor -> LGSVLState. Works for real and mock actors alike."""
    tf = carla_actor.get_transform()
    vel = carla_actor.get_velocity()
    ang = carla_actor.get_angular_velocity()

    position = _V3(tf.location.x, tf.location.z, -tf.location.y)
    # rotation.y carries heading in degrees (LGSVL convention)
    rotation = _V3(0.0, -tf.rotation.yaw, 0.0)
    transform = _Transform(position, rotation)

    velocity = _V3(vel.x, 0.0, -vel.y)
    angular_velocity = _V3(ang.x, ang.z, -ang.y)

    return LGSVLState(transform, velocity, angular_velocity)


def bbox_from_carla(carla_actor):
    """carla.Actor.bounding_box (half-extents) -> LGSVLBBox (min/max local)."""
    e = carla_actor.bounding_box.extent
    # LGSVL local axes: x=length(fwd), y=height(up), z=width(lateral)
    min_v = _V3(-e.x, 0.0, -e.y)
    max_v = _V3(e.x, e.z * 2.0, e.y)
    return LGSVLBBox(min_v, max_v)


def location_to_dest(carla_location):
    """carla.Location -> object with .x/.z for CaseRecorder.set_destination."""
    class _Dest(object):
        __slots__ = ("x", "z")
    d = _Dest()
    d.x = carla_location.x
    d.z = -carla_location.y
    return d
