#!/usr/bin/env python3
"""Feed vehicle_controller an endless series of random waypoints.

Each waypoint is drawn uniformly from x, y in [-bound, bound] and kept only
if it is at least wall_clearance from every wall random_cube_spawner has
placed. It goes to vehicle_controller on goal_topic, and a small green disc
marks it on the ground in Gazebo. Once the vehicle's center is within
reach_radius of it, a new one is drawn by the same rules, and so on for as
long as the node runs.

Where the walls are
-------------------
random_cube_spawner publishes the footprint of every wall it has requested on
walls_topic (see walls_to_msg there). That topic is transient local, so this
node gets the full list however late it starts. No waypoint is drawn before
the first list arrives. A wall added later through ~/spawn_cube republishes
the list, and a current waypoint that the new wall crowds is replaced.

When a new waypoint is drawn
----------------------------
  - the vehicle's center comes within reach_radius of the current one
    (from odom_topic). Keep this equal to vehicle_controller's
    goal_tolerance, which is where the vehicle stops.
  - vehicle_controller reports "unreachable" on state_topic. Bug2 has given
    up on this waypoint and would otherwise sit still forever.
  - a new wall list puts a wall within wall_clearance of it.

A candidate within min_travel of the vehicle is also rejected, so the next
waypoint is never one the vehicle is already sitting on.

Handing goals to the controller
-------------------------------
The controller's goal subscription is volatile: a goal published before it
subscribes is lost. So the current waypoint is also re-sent whenever a new
subscriber from a node named controller_name appears. Only that node counts;
a `ros2 topic echo` on the goal topic must not re-send the goal, because every
goal the controller receives restarts Bug2 from wherever the vehicle is.

The marker
----------
A static, visual-only model (no collision, so the vehicle drives over it)
spawned once through /world/<world_name>/create and then moved with
/world/<world_name>/set_pose. It is a few millimeters tall, far below the
lidar's scan plane, so the vehicle never sees it.

Parameters
----------
world_name : str        Must match <world name="..."> in the SDF. Default
                        "spawn_demo".
bound : float           Waypoints land in x, y uniform over [-bound, bound].
                        Default 12.0.
wall_clearance : float  Minimum distance from a waypoint to any wall's
                        footprint, in meters. Default 1.5.
reach_radius : float    The vehicle has reached a waypoint once its center is
                        this close, in meters. Default 1.0.
min_travel : float      Reject waypoints closer than this to the vehicle, in
                        meters. Default 2.0.
seed : int              RNG seed. -1 (default) means seed from system entropy.
goal_topic : str        Default "/vehicle/goal".
odom_topic : str        Default "/vehicle/odom".
walls_topic : str       Default "/walls".
state_topic : str       Default "/vehicle/nav_state".
controller_name : str   Node whose goal subscription triggers a re-send.
                        Default "vehicle_controller".
marker_name : str       Gazebo model name of the marker. Default
                        "waypoint_marker".
marker_radius : float   Radius of the marker disc, in meters. Default 0.3.
"""

import math
import random

from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from std_msgs.msg import Float64MultiArray, String

from waypoint_nav.random_cube_spawner import (
    footprint_distance_to_origin,
    walls_from_msg,
)

# Resampling budget per waypoint. Only runs out if walls and their clearance
# cover nearly the whole area.
MAX_WAYPOINT_TRIES = 10000

# Marker disc thickness, in meters. Thin enough to read as a mark on the
# ground, thick enough not to z-fight with the ground plane.
MARKER_HEIGHT = 0.02


def distance_to_wall(point, wall) -> float:
    """Distance from point (x, y) to a wall footprint (x, y, yaw, sx, sy).

    Zero if the point is inside the footprint.
    """
    x, y, yaw, sx, sy = wall
    # footprint_distance_to_origin measures from the origin; shift the whole
    # picture so point is the origin.
    return footprint_distance_to_origin(
        x - point[0], y - point[1], yaw, sx, sy)


def waypoint_is_valid(point, walls, clearance, bound,
                      vehicle=None, min_travel=0.0) -> bool:
    """Whether point is in bounds, clear of walls, and away from the vehicle."""
    if not (-bound <= point[0] <= bound and -bound <= point[1] <= bound):
        return False
    if vehicle is not None and math.hypot(
            point[0] - vehicle[0], point[1] - vehicle[1]) < min_travel:
        return False
    return all(distance_to_wall(point, wall) >= clearance for wall in walls)


def pick_waypoint(rng, walls, clearance, bound, vehicle=None, min_travel=0.0):
    """Sample a valid waypoint as (x, y), or None if every try failed."""
    for _ in range(MAX_WAYPOINT_TRIES):
        point = (rng.uniform(-bound, bound), rng.uniform(-bound, bound))
        if waypoint_is_valid(point, walls, clearance, bound,
                             vehicle, min_travel):
            return point
    return None


def marker_sdf(name: str, radius: float) -> str:
    """Describe the marker: a static, visual-only green disc on the ground."""
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <pose>0 0 {MARKER_HEIGHT / 2.0} 0 0 0</pose>
        <geometry>
          <cylinder><radius>{radius}</radius><length>{MARKER_HEIGHT}</length></cylinder>
        </geometry>
        <cast_shadows>false</cast_shadows>
        <material>
          <ambient>0.1 0.8 0.2 1</ambient>
          <diffuse>0.1 0.8 0.2 1</diffuse>
          <emissive>0.05 0.4 0.1 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def ground_pose(point) -> Pose:
    pose = Pose()
    pose.position.x = float(point[0])
    pose.position.y = float(point[1])
    pose.orientation.w = 1.0
    return pose


class WaypointGenerator(Node):

    def __init__(self):
        super().__init__('waypoint_generator')

        self.declare_parameter('world_name', 'spawn_demo')
        self.declare_parameter('bound', 12.0)
        self.declare_parameter('wall_clearance', 1.5)
        self.declare_parameter('reach_radius', 1.0)
        self.declare_parameter('min_travel', 2.0)
        self.declare_parameter('seed', -1)
        self.declare_parameter('goal_topic', '/vehicle/goal')
        self.declare_parameter('odom_topic', '/vehicle/odom')
        self.declare_parameter('walls_topic', '/walls')
        self.declare_parameter('state_topic', '/vehicle/nav_state')
        self.declare_parameter('controller_name', 'vehicle_controller')
        self.declare_parameter('marker_name', 'waypoint_marker')
        self.declare_parameter('marker_radius', 0.3)

        def number(name):
            return float(self.get_parameter(name).value)

        self.world_name = self.get_parameter('world_name').value
        self.bound = number('bound')
        self.wall_clearance = number('wall_clearance')
        self.reach_radius = number('reach_radius')
        self.min_travel = number('min_travel')
        self.goal_topic = self.get_parameter('goal_topic').value
        self.controller_name = self.get_parameter('controller_name').value
        self.marker_name = self.get_parameter('marker_name').value
        self.marker_radius = number('marker_radius')

        seed = self.get_parameter('seed').value
        if seed < 0:
            seed = random.randrange(2 ** 31)
        self.rng = random.Random(seed)
        self.get_logger().info(f'RNG seed: {seed}')

        # None until the first wall list; the waypoint is None until then too.
        self.walls = None
        self.waypoint = None
        self.waypoint_count = 0
        # (x, y) of the vehicle, None until the first odometry.
        self.vehicle = None
        # GIDs of the controller's goal subscriptions already sent the goal.
        self._known_controllers = set()
        # Marker lifecycle: 'absent' -> 'spawning' -> 'spawned'. Where the
        # marker is (or will be once a request lands), to skip no-op moves.
        self._marker_state = 'absent'
        self._marker_at = None
        self._marker_busy = False

        self.goal_pub = self.create_publisher(Point, self.goal_topic, 10)

        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Float64MultiArray, self.get_parameter('walls_topic').value,
            self._on_walls, latched)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            String, self.get_parameter('state_topic').value,
            self._on_state, 10)

        self.spawn_cli = self.create_client(
            SpawnEntity, f'/world/{self.world_name}/create')
        self.pose_cli = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose')
        # Service readiness and new controller subscriptions are both things
        # to poll for; neither has a callback.
        self.create_timer(0.5, self._on_timer)

        self.get_logger().info(
            f'Waypoints in [-{self.bound:.1f}, {self.bound:.1f}]^2, at least '
            f'{self.wall_clearance:.2f} m from walls, reached within '
            f'{self.reach_radius:.2f} m, published on {self.goal_topic}. '
            'Waiting for the wall list.')

    def next_waypoint(self, reason: str):
        """Replace the current waypoint and hand it to the controller."""
        point = pick_waypoint(
            self.rng, self.walls, self.wall_clearance, self.bound,
            self.vehicle, self.min_travel)
        if point is None:
            self.get_logger().error(
                f'No waypoint clear of {len(self.walls)} walls after '
                f'{MAX_WAYPOINT_TRIES} tries; keeping the current one',
                throttle_duration_sec=5.0)
            return
        self.waypoint = point
        self.waypoint_count += 1
        self.get_logger().info(
            f'Waypoint {self.waypoint_count} at ({point[0]:.2f}, '
            f'{point[1]:.2f}) ({reason})')
        self.publish_goal()
        self.update_marker()

    def publish_goal(self):
        if self.waypoint is None:
            return
        self.goal_pub.publish(
            Point(x=float(self.waypoint[0]), y=float(self.waypoint[1])))

    def update_marker(self):
        """Put the marker on the current waypoint, spawning it if needed.

        At most one request is in flight at a time; whichever finishes checks
        whether the waypoint moved meanwhile and follows up.
        """
        if self.waypoint is None or self._marker_busy:
            return
        if self._marker_state == 'absent':
            if not self.spawn_cli.service_is_ready():
                return  # the timer retries
            self._request_spawn()
        elif self._marker_at != self.waypoint:
            if not self.pose_cli.service_is_ready():
                return
            self._request_move()

    def _request_spawn(self):
        factory = EntityFactory()
        factory.name = self.marker_name
        factory.allow_renaming = False
        factory.sdf = marker_sdf(self.marker_name, self.marker_radius)
        factory.pose = ground_pose(self.waypoint)
        factory.relative_to = 'world'
        request = SpawnEntity.Request()
        request.entity_factory = factory

        self._marker_state = 'spawning'
        self._marker_busy = True
        self._marker_at = self.waypoint
        self.spawn_cli.call_async(request).add_done_callback(
            self._on_spawn_done)

    def _on_spawn_done(self, future):
        self._marker_busy = False
        try:
            success = future.result().success
        except Exception as exc:  # noqa: BLE001 - want the reason in the log
            self.get_logger().error(f'Marker spawn raised: {exc}')
            success = False
        if not success:
            # Most likely a marker left over from an earlier run of this node
            # in the same world. Moving that one is just as good.
            self.get_logger().warning(
                f'Gazebo refused to spawn {self.marker_name}; assuming it '
                'already exists and moving it instead')
        self._marker_state = 'spawned'
        if not success:
            self._marker_at = None
        self.update_marker()

    def _request_move(self):
        request = SetEntityPose.Request()
        request.entity = Entity(name=self.marker_name, type=Entity.MODEL)
        request.pose = ground_pose(self.waypoint)
        self._marker_busy = True
        target = self.waypoint
        self.pose_cli.call_async(request).add_done_callback(
            lambda future: self._on_move_done(future, target))

    def _on_move_done(self, future, target):
        self._marker_busy = False
        try:
            success = future.result().success
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Marker move raised: {exc}')
            success = False
        if success:
            self._marker_at = target
        else:
            self.get_logger().warning(
                f'Gazebo refused to move {self.marker_name}; will retry',
                throttle_duration_sec=5.0)
            return  # the timer retries, rather than spinning on failures
        self.update_marker()

    def _on_timer(self):
        self._resend_to_new_controllers()
        self.update_marker()

    def _resend_to_new_controllers(self):
        gids = {
            bytes(info.endpoint_gid)
            for info in self.get_subscriptions_info_by_topic(self.goal_topic)
            if info.node_name == self.controller_name
        }
        new = gids - self._known_controllers
        # Forget controllers that went away, so a restarted one is new again.
        self._known_controllers = gids
        if new and self.waypoint is not None:
            self.get_logger().info(
                f'{self.controller_name} subscribed to {self.goal_topic}; '
                'sending it the current waypoint')
            self.publish_goal()

    def _on_walls(self, msg: Float64MultiArray):
        self.walls = walls_from_msg(msg)
        self.get_logger().info(f'Wall list: {len(self.walls)} walls')
        if self.waypoint is None:
            self.next_waypoint('first waypoint')
        elif not waypoint_is_valid(self.waypoint, self.walls,
                                   self.wall_clearance, self.bound):
            self.next_waypoint('a new wall is too close to the last one')

    def _on_odom(self, msg: Odometry):
        position = msg.pose.pose.position
        self.vehicle = (position.x, position.y)
        if self.waypoint is None:
            return
        distance = math.hypot(self.waypoint[0] - position.x,
                              self.waypoint[1] - position.y)
        if distance <= self.reach_radius:
            self.next_waypoint(
                f'reached the last one, {distance:.2f} m from its center')

    def _on_state(self, msg: String):
        if msg.data == 'unreachable' and self.waypoint is not None:
            self.next_waypoint('the controller gave up on the last one')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
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
