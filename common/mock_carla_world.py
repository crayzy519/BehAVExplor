"""
Mock of the real CARLA object API (carla.World / carla.Actor / carla.Transform).

Why this exists
---------------
We are migrating BehAVExplor to drive CARLA, but CARLA is not running yet. To
develop and test the *real* migration code (the CarlaStateAdapter in
common/carla_adapter.py) without a live simulator, we emulate the small slice of
the CARLA Python API that the bridge actually touches.

Faithfulness to real CARLA
---------------------------
Field names / call shapes mirror the real API exactly, as used in
carla_bridge/actor/*.py:
    actor.get_transform()          -> Transform(.location, .rotation)
        .location.{x,y,z}          (metres, CARLA left-handed frame)
        .rotation.{pitch,yaw,roll} (DEGREES)
    actor.get_velocity()           -> Vector3D(.x,.y,.z)   (m/s)
    actor.get_angular_velocity()   -> Vector3D             (deg/s)
    actor.get_acceleration()       -> Vector3D             (m/s^2)
    actor.bounding_box.extent.{x,y,z}   (HALF extents, metres)
    world.tick() / world.get_snapshot().timestamp.elapsed_seconds

So swapping in the real CARLA (client.get_world(), world.spawn_actor(...)) later
requires NO change to carla_adapter.py or anything above it. Only the producer of
these objects changes: MockCarlaWorld -> carla.World.

CARLA coordinate frame note
---------------------------
Real CARLA is left-handed (Y points right/down). The bridge flips Y when leaving
CARLA (see carla_bridge/actor/traffic_participant.py:78 `-location.y`). We keep
Mock actors in the *raw CARLA frame* (no flip here) so the adapter is the single
place that performs the CARLA->world flip -- exactly like the real bridge.
"""
import math


class Vector3D(object):
    """Mirror of carla.Vector3D / carla.Location."""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)


# carla.Location is a Vector3D subclass in the real API; alias is enough here.
Location = Vector3D


class Rotation(object):
    """Mirror of carla.Rotation. Angles in DEGREES, like real CARLA."""

    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch = float(pitch)
        self.yaw = float(yaw)
        self.roll = float(roll)


class Transform(object):
    """Mirror of carla.Transform."""

    def __init__(self, location=None, rotation=None):
        self.location = location or Location()
        self.rotation = rotation or Rotation()


class BoundingBox(object):
    """Mirror of carla.BoundingBox. `extent` holds HALF dimensions (metres)."""

    def __init__(self, extent=None):
        # default ~ a sedan: 4.5 x 2.0 x 1.5 m -> half extents
        self.extent = extent or Vector3D(2.25, 1.0, 0.75)


class MockCarlaActor(object):
    """
    Emulates carla.Actor for the getters the bridge reads.

    A trajectory (list of (Transform, Vector3D velocity, Vector3D ang_vel)) is
    fed in; `world.tick()` advances the internal frame pointer. This is the ONLY
    thing that differs from a real actor -- a real actor's physics engine updates
    these; here we replay a scripted trajectory.
    """

    _next_id = 1

    def __init__(self, trajectory, bounding_box=None):
        self.id = MockCarlaActor._next_id
        MockCarlaActor._next_id += 1
        self._trajectory = trajectory  # list of dict(transform, velocity, ang_vel, accel)
        self._frame = 0
        self.bounding_box = bounding_box or BoundingBox()

    # ---- frame advance (driven by MockCarlaWorld.tick) -------------------- #
    def _advance(self):
        if self._frame < len(self._trajectory) - 1:
            self._frame += 1

    def _cur(self):
        return self._trajectory[min(self._frame, len(self._trajectory) - 1)]

    def is_finished(self):
        return self._frame >= len(self._trajectory) - 1

    # ---- real carla.Actor getters ---------------------------------------- #
    def get_transform(self):
        return self._cur()["transform"]

    def get_location(self):
        return self._cur()["transform"].location

    def get_velocity(self):
        return self._cur().get("velocity", Vector3D())

    def get_angular_velocity(self):
        return self._cur().get("ang_vel", Vector3D())

    def get_acceleration(self):
        return self._cur().get("accel", Vector3D())


class MockCollisionSensor(object):
    """
    Mirror of a CARLA collision sensor (`sensor.other.collision`).

    Real CARLA usage (carla_bridge/core/actor_factory.py:377-382):
        bp     = blueprint_library.find('sensor.other.collision')
        sensor = world.spawn_actor(bp, Transform(), attach_to=ego)
        sensor.listen(lambda event: ...)   # event.other_actor = what we hit
    The engine calls the listener on every physical contact.

    We have no physics engine, so this mock NEVER fires -- the collision flag
    stays False, exactly like the current hardcoded value. Only the *plumbing*
    is real: swap this for a spawned carla sensor later and the run loop that
    registers the callback + reads the flag does not change.
    """

    def __init__(self, parent_actor):
        self.parent = parent_actor
        self._callback = None

    def listen(self, callback):
        self._callback = callback

    def stop(self):
        self._callback = None

    def destroy(self):
        self._callback = None


class _Timestamp(object):
    def __init__(self, elapsed_seconds):
        self.elapsed_seconds = elapsed_seconds


class _Snapshot(object):
    def __init__(self, elapsed_seconds):
        self.timestamp = _Timestamp(elapsed_seconds)


class MockCarlaWorld(object):
    """
    Emulates the slice of carla.World the run loop uses:
        world.tick()                                  -> advance one fixed step
        world.get_snapshot().timestamp.elapsed_seconds
    Actors registered via add_actor() advance together on each tick.
    """

    def __init__(self, dt=0.1):
        self.dt = dt
        self._elapsed = 0.0
        self._actors = []

    def add_actor(self, actor):
        self._actors.append(actor)
        return actor

    def spawn_collision_sensor(self, parent_actor):
        """Mirror of world.spawn_actor(collision_bp, ..., attach_to=parent)."""
        return MockCollisionSensor(parent_actor)

    def tick(self):
        for a in self._actors:
            a._advance()
        self._elapsed += self.dt
        return self._elapsed

    def get_snapshot(self):
        return _Snapshot(self._elapsed)

    def all_finished(self):
        return all(a.is_finished() for a in self._actors)
