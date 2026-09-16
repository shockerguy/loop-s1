#!/usr/bin/env python3
"""Spawn rectangular-prism walls at random ground-plane poses at runtime.

Each wall gets a random length along its local x axis, a fixed thickness (y)
and height (z), and a random yaw about the vertical axis.

Talks to Gazebo's /world/<world>/create service, bridged into ROS 2 as
ros_gz_interfaces/srv/SpawnEntity by ros_gz_bridge.

Parameters
----------
world_name : str      Must match <world name="..."> in the SDF. Default "spawn_demo".
count : int           How many walls to spawn on startup. Default 1.
seed : int            RNG seed. -1 (default) means seed from system entropy.
bound : float         Walls land in x, y uniform over [-bound, bound]. Default 10.0.
size_x_min : float    Wall length is uniform over [size_x_min, size_x_max].
size_x_max : float    Defaults 1.0 and 4.0.
size_y : float        Wall thickness. Default 1.0.
size_z : float        Wall height. Default 2.0 (so the center sits at z = 1.0).
static : bool         True (default) makes walls immovable, which is what you
                      want for obstacles. False lets the vehicle shove them.
clear_radius : float  No part of a wall's footprint lands within this distance
                      of the origin, so walls can't spawn on top of the
                      vehicle. Default 0.0 (no keep-out zone).

Walls never overlap each other. A wall that can't be placed within
MAX_PLACEMENT_TRIES samples is skipped with a warning rather than forced in.

Also advertises ~/spawn_cube (std_srvs/Trigger) so you can add more walls
without restarting the sim.
"""

import math
import random

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityFactory
from ros_gz_interfaces.srv import SpawnEntity
from std_srvs.srv import Trigger

# Resampling budget for clear_radius and wall-wall overlap. Only runs out if
# the keep-out zone and existing walls cover nearly the whole spawn area.
MAX_PLACEMENT_TRIES = 1000


def footprint_distance_to_origin(
        x: float, y: float, yaw: float, sx: float, sy: float) -> float:
    """Distance from the origin to a yawed sx-by-sy rectangle centered at (x, y).

    Zero if the origin is inside the rectangle.
    """
    # Origin expressed in the rectangle's frame: rotate (-x, -y) by -yaw.
    c, s = math.cos(yaw), math.sin(yaw)
    lx = -c * x - s * y
    ly = s * x - c * y
    dx = max(abs(lx) - sx / 2.0, 0.0)
    dy = max(abs(ly) - sy / 2.0, 0.0)
    return math.hypot(dx, dy)


def footprints_overlap(a: tuple, b: tuple) -> bool:
    """Whether two yawed rectangles (x, y, yaw, sx, sy) share any area.

    Separating axis test: two convex shapes are disjoint iff their
    projections onto some edge normal don't overlap. Rectangles touching
    along an edge or corner don't count as overlapping.
    """
    dx, dy = b[0] - a[0], b[1] - a[1]
    axes = []
    for _, _, yaw, _, _ in (a, b):
        c, s = math.cos(yaw), math.sin(yaw)
        axes += [(c, s), (-s, c)]

    for nx, ny in axes:
        # Half-extent of each rectangle projected onto the axis.
        radius = 0.0
        for _, _, yaw, sx, sy in (a, b):
            c, s = math.cos(yaw), math.sin(yaw)
            radius += (sx / 2.0) * abs(c * nx + s * ny)
            radius += (sy / 2.0) * abs(-s * nx + c * ny)
        if abs(dx * nx + dy * ny) >= radius:
            return False
    return True


def prism_sdf(name: str, sx: float, sy: float, sz: float, static: bool) -> str:
    mass = 1.0
    # Solid cuboid about its center: I_xx = m * (y^2 + z^2) / 12, etc.
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0

    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>{'true' if static else 'false'}</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy}</iyy><iyz>0</iyz>
          <izz>{izz}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material>
          <ambient>0.3 0.4 0.7 1</ambient>
          <diffuse>0.3 0.4 0.7 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class RandomCubeSpawner(Node):

    def __init__(self):
        super().__init__('random_cube_spawner')

        self.declare_parameter('world_name', 'spawn_demo')
        self.declare_parameter('count', 1)
        self.declare_parameter('seed', -1)
        self.declare_parameter('bound', 10.0)
        self.declare_parameter('size_x_min', 1.0)
        self.declare_parameter('size_x_max', 4.0)
        self.declare_parameter('size_y', 1.0)
        self.declare_parameter('size_z', 2.0)
        self.declare_parameter('static', True)
        self.declare_parameter('clear_radius', 0.0)

        self.world_name = self.get_parameter('world_name').value
        self.bound = self.get_parameter('bound').value
        self.size_x_min = self.get_parameter('size_x_min').value
        self.size_x_max = self.get_parameter('size_x_max').value
        self.size_y = self.get_parameter('size_y').value
        self.size_z = self.get_parameter('size_z').value
        self.static = self.get_parameter('static').value
        self.clear_radius = self.get_parameter('clear_radius').value

        seed = self.get_parameter('seed').value
        if seed < 0:
            seed = random.randrange(2 ** 31)
        self.rng = random.Random(seed)
        # Log the seed unconditionally: a run you can't reproduce is a run you
        # can't debug, and this is the only place the value exists.
        self.get_logger().info(f'RNG seed: {seed}')

        self._spawn_count = 0
        # name -> (x, y, yaw, sx, sy) of every wall requested so far, so new
        # walls can be kept off existing ones.
        self._footprints = {}
        service_name = f'/world/{self.world_name}/create'
        self.cli = self.create_client(SpawnEntity, service_name)

        self.create_service(Trigger, '~/spawn_cube', self._on_trigger)

        self.get_logger().info(f'Waiting for {service_name} ...')
        self._pending = self.get_parameter('count').value
        self._wait_timer = self.create_timer(0.5, self._wait_for_service)

    def _wait_for_service(self):
        if not self.cli.service_is_ready():
            return
        self._wait_timer.cancel()
        self.get_logger().info('Service is up.')
        for _ in range(self._pending):
            self.spawn_random_cube()

    def _on_trigger(self, request, response):
        if not self.cli.service_is_ready():
            response.success = False
            response.message = 'Spawn service not available'
            return response
        name = self.spawn_random_cube()
        if name is None:
            response.success = False
            response.message = 'No free placement found; see node log'
            return response
        response.success = True
        # Fire-and-forget: the Gazebo call runs asynchronously, so this only
        # confirms the request went out, not that the wall exists yet.
        response.message = f'Requested spawn of {name}'
        return response

    def pick_placement(self):
        """Sample (x, y, yaw, size_x) clear of the origin and existing walls.

        Returns None if MAX_PLACEMENT_TRIES samples all fail.
        """
        for _ in range(MAX_PLACEMENT_TRIES):
            size_x = self.rng.uniform(self.size_x_min, self.size_x_max)
            yaw = self.rng.uniform(-math.pi, math.pi)
            x = self.rng.uniform(-self.bound, self.bound)
            y = self.rng.uniform(-self.bound, self.bound)
            if footprint_distance_to_origin(
                    x, y, yaw, size_x, self.size_y) < self.clear_radius:
                continue
            candidate = (x, y, yaw, size_x, self.size_y)
            if any(footprints_overlap(candidate, other)
                   for other in self._footprints.values()):
                continue
            return x, y, yaw, size_x
        return None

    def spawn_random_cube(self):
        """Request one wall. Returns its name, or None if it didn't fit."""
        self._spawn_count += 1
        name = f'random_wall_{self._spawn_count}'

        placement = self.pick_placement()
        if placement is None:
            self.get_logger().warning(
                f'Skipping {name}: no placement clear of clear_radius '
                f'{self.clear_radius:.2f} m and the {len(self._footprints)} '
                f'existing walls after {MAX_PLACEMENT_TRIES} tries')
            return None
        x, y, yaw, size_x = placement
        self._footprints[name] = (x, y, yaw, size_x, self.size_y)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = self.size_z / 2.0  # rest the wall on the ground
        # Pure rotation about z: q = (0, 0, sin(yaw/2), cos(yaw/2))
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        factory = EntityFactory()
        factory.name = name
        factory.allow_renaming = True
        factory.sdf = prism_sdf(
            name, size_x, self.size_y, self.size_z, self.static)
        factory.pose = pose
        factory.relative_to = 'world'

        req = SpawnEntity.Request()
        req.entity_factory = factory

        future = self.cli.call_async(req)
        future.add_done_callback(
            lambda fut, n=name, p=pose, sx=size_x, y=yaw:
                self._on_spawn_done(fut, n, p, sx, y))
        return name

    def _on_spawn_done(self, future, name, pose, size_x, yaw):
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - want the reason in the log
            self._footprints.pop(name, None)
            self.get_logger().error(f'Spawn of {name} raised: {exc}')
            return

        if result.success:
            self.get_logger().info(
                f'Spawned {name} at '
                f'({pose.position.x:.2f}, {pose.position.y:.2f}, '
                f'{pose.position.z:.2f}), '
                f'size ({size_x:.2f}, {self.size_y:.2f}, {self.size_z:.2f}), '
                f'yaw {math.degrees(yaw):.1f} deg')
        else:
            self._footprints.pop(name, None)
            self.get_logger().warning(
                f'Gazebo refused to spawn {name} (check for a name collision '
                f'or malformed SDF)')


def main(args=None):
    rclpy.init(args=args)
    node = RandomCubeSpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
