#!/usr/bin/env python3
"""Drive the vehicle to a goal point with the Bug2 navigation algorithm.

The vehicle knows two things: where it is (/vehicle/odom) and how far away
whatever is in front of it happens to be (/vehicle/depths, the cleaned lidar
fan from lidar_scanner). Bug2 is enough to get from one to the other in an
environment this simple.

The algorithm
-------------
The *m-line* is the straight segment from where the vehicle started to the
goal. Bug2 alternates between following it and following a wall:

  GO_TO_GOAL    Turn onto the bearing of the goal and drive. If the forward
                sector becomes obstructed, remember this spot as the hit
                point, together with its distance to the goal, and start
                wall following.
  TURN_CCW      Rotate counter-clockwise in place until the forward sector is
                clear by obstacle_distance plus clear_margin, then start
                following the wall. The margin is hysteresis: leaving at
                exactly the threshold makes the vehicle drive a few
                centimeters, trip the threshold again and chatter.
  FOLLOW_WALL   The wall is off to starboard, because turning CCW is what put
                it there. Each tick, one of three things holds:
                  - forward obstructed: go back to TURN_CCW.
                  - right clear: curve clockwise (an arc of turn_radius) to
                    hunt for the wall that just ended.
                  - forward clear and right obstructed: drive straight. This
                    is the steady state of wall following.
                With one refinement the bare algorithm needs on a vehicle
                that has width: inside min_side_distance the wall is close
                enough that the corners would catch on it during the next
                turn in place, so the vehicle drives on but bends away from
                it (a counter-clockwise arc) until it has room again.
                Wall following ends when the vehicle is back on the m-line
                *and* closer to the goal than it was at the hit point. The
                second half of that is what stops it from leaving the line at
                the same place forever.
  BACK_UP       Not part of Bug2, and not reached from any of its rules.
                Bug2 assumes a point that can always turn on the spot; a
                2 x 1 m box in a corner cannot, and sits there commanding a
                turn that physics refuses. So when the vehicle has been
                commanded to move but has gone nowhere for stuck_timeout, it
                reverses backup_distance - blind, but into ground it just
                drove over - and resumes whatever it was doing from there.
  REACHED       The vehicle's center is within goal_tolerance of the goal.
                Default tolerance is 1 m, the width of the vehicle.
  UNREACHABLE   The vehicle wall-followed all the way back to the hit point
                without ever finding a better crossing of the m-line, so the
                obstacle encircles either it or the goal. It stops; publish a
                new goal on goal_topic to send it somewhere else.

                Or it gave up: Bug2 only promises to terminate while it
                follows one obstacle's boundary, and among scattered walls
                the vehicle hands off from one to the next and can end up
                circling a group of them on a circuit that never passes the
                hit point. Nothing in the algorithm notices. So wall
                following also gets a budget, follow_limit, measured in
                meters travelled since leaving the m-line.

Sectors
-------
The depth array is split into three sectors by bearing, and each one is
summarized by its closest beam:

    right    -90 .. -20 deg
    forward  -20 ..  20 deg
    left      20 ..  90 deg

A narrow forward sector keeps a wall the vehicle is driving *past* from
reading as a wall it is about to hit. The boundary beams fall in both of the
sectors that share them, which is close enough at 1 deg per beam. The left
sector is unused by right-hand wall following; it is computed and logged
because a mirrored version of this algorithm would want it.

Only depths and odometry drive any of this - there is no map, and the vehicle
never plans further ahead than the sector it is looking at.

Parameters
----------
cmd_vel_topic : str     Twist topic the vehicle listens on. Default
                        "/vehicle/cmd_vel".
depths_topic : str      Depth array from lidar_scanner. Default
                        "/vehicle/depths".
odom_topic : str        Odometry from the vehicle. Default "/vehicle/odom".
goal_topic : str        geometry_msgs/Point topic that reassigns the goal at
                        runtime. Default "/vehicle/goal".
rate : float            Publish rate in Hz. Default 20.0.
goal_x, goal_y : float  Goal position in the odom frame. Default (8.0, 0.0).
goal_tolerance : float  Goal counts as reached inside this radius, measured
                        from the center of the vehicle. Default 1.0, the
                        width of the vehicle.
linear_speed : float    Forward speed in m/s. Default 1.0.
turn_speed : float      Yaw rate for turning in place, rad/s. Default 0.6.
turn_radius : float     Radius in meters of the clockwise arc used to hunt
                        for a wall that has ended. Default 2.0.
obstacle_distance : float  Forward sector is obstructed below this, in
                        meters. Default 1.5.
clear_margin : float    Extra clearance, in meters, the forward sector needs
                        before a CCW turn hands back to wall following.
                        Hysteresis; 0 disables it. Default 0.5.
wall_distance : float   Right sector is obstructed below this, in meters.
                        Default 1.5.
min_side_distance : float  Below this, the wall to starboard is too close to
                        turn beside, and the vehicle bends away from it while
                        driving on. 0 disables that. Default 0.8 m, which
                        clears the 1.118 m the corners of a 2 x 1 m body
                        sweep when it rotates in place. Must be under
                        wall_distance.
line_tolerance : float  How close to the m-line still counts as on it, in
                        meters. Default 0.3.
progress_epsilon : float  Rejoining the m-line only counts if it happens
                        this much closer to the goal than the hit point.
                        Default 0.3 m.
align_tolerance_deg : float  Heading error above which the vehicle turns in
                        place instead of steering while driving. Default 20.0.
heading_gain : float    Proportional gain from heading error (rad) to yaw
                        rate while driving. Default 1.5.
max_yaw_rate : float    Cap on the yaw rate that gain can ask for, rad/s.
                        Default 1.0.
loop_clear_distance : float  The vehicle has to get this far from the hit
                        point before returning to it means anything.
                        Default 1.5 m.
loop_tolerance : float  Coming back within this distance of the hit point,
                        after having left it, means the goal is unreachable.
                        Default 0.5 m.
follow_limit : float    Give up after wall following this many meters without
                        rejoining the m-line. 0 for no limit. Default 60.0,
                        about three times the width of the demo world; a
                        bigger world wants a bigger number.
forward_half_angle_deg : float  Half-width of the forward sector. Default 20.0.
side_max_angle_deg : float  Outer edge of the side sectors. Default 90.0.
lidar_fov_deg : float   Total field of view of the depth array, used to turn
                        an index into a bearing. Must match
                        <min_angle>/<max_angle> in models/vehicle/model.sdf.
                        Default 180.0.
stuck_timeout : float   Seconds of being commanded to move while going
                        nowhere before the vehicle reverses out. 0 disables
                        the escape entirely. Default 2.0.
stuck_distance : float  Movement under this over stuck_timeout is going
                        nowhere, in meters. Default 0.25.
stuck_angle_deg : float  Rotation under this over stuck_timeout is going
                        nowhere, in degrees. Default 25.0.
backup_distance : float  How far to reverse when stuck, in meters.
                        Default 0.8.
backup_speed : float    How fast to reverse, in m/s. Default 0.5.
log_period : float      Seconds between status log lines; 0 disables them.
                        Default 2.0.
"""

import enum
import math
import signal

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray


def beam_bearing_deg(index: int, count: int, fov_deg: float) -> float:
    """Bearing of one beam in a symmetric fan, in degrees.

    Negative is starboard (the vehicle's right), 0 straight ahead, positive
    port. The fan spans the full fov_deg, so the first and last beams sit on
    its edges.
    """
    if count < 2:
        return 0.0
    return -fov_deg / 2.0 + index * fov_deg / (count - 1)


def sector_min(depths, fov_deg: float, lo_deg: float, hi_deg: float) -> float:
    """Closest depth among the beams bearing within [lo_deg, hi_deg].

    Both bounds are inclusive, so a beam sitting exactly on the boundary
    between two sectors is reported in both. Returns inf when no beam falls
    in the sector, which reads as "nothing there" to every caller.
    """
    count = len(depths)
    closest = math.inf
    for index, depth in enumerate(depths):
        bearing = beam_bearing_deg(index, count, fov_deg)
        if lo_deg <= bearing <= hi_deg and depth < closest:
            closest = depth
    return closest


def normalize_angle(angle: float) -> float:
    """Wrap an angle in radians into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw in radians from a quaternion, ignoring roll and pitch.

    The vehicle drives on flat ground, so yaw is the whole of its attitude.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def distance_to_line(point, start, goal) -> float:
    """Perpendicular distance from point to the infinite line start -> goal.

    Falls back to the distance to start when start and goal coincide, which
    is the only sensible reading of a zero-length m-line.
    """
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    # Cross product of the line direction with the start -> point vector.
    return abs(dx * (point[1] - start[1]) - dy * (point[0] - start[0])) / length


def clamp(value: float, limit: float) -> float:
    """Clamp value into [-limit, limit]."""
    return max(-limit, min(limit, value))


class Bug2State(enum.Enum):
    """Where the vehicle is in the Bug2 cycle. See the module docstring."""

    GO_TO_GOAL = 'go_to_goal'
    TURN_CCW = 'turn_ccw'
    FOLLOW_WALL = 'follow_wall'
    BACK_UP = 'back_up'
    REACHED = 'reached'
    UNREACHABLE = 'unreachable'


class Bug2Planner:
    """Bug2 state machine over a pose and three sector depths.

    Deliberately free of ROS types: update() takes (x, y, yaw) and
    (right, forward, left) depths in meters and returns the body-frame
    (linear_x, angular_z) to command. That keeps the whole algorithm testable
    without a running graph, and keeps VehicleController down to plumbing.
    """

    def __init__(self, goal, *, linear_speed=1.0, turn_speed=0.6,
                 turn_radius=2.0, goal_tolerance=1.0, obstacle_distance=1.5,
                 clear_margin=0.5, wall_distance=1.5, min_side_distance=0.8,
                 line_tolerance=0.3, progress_epsilon=0.3,
                 align_tolerance_deg=20.0, heading_gain=1.5, max_yaw_rate=1.0,
                 loop_clear_distance=1.5, loop_tolerance=0.5,
                 follow_limit=60.0, stuck_timeout=2.0, stuck_distance=0.25, stuck_angle_deg=25.0,
                 backup_distance=0.8, backup_speed=0.5):
        if turn_radius == 0.0:
            raise ValueError('turn_radius must be nonzero')
        if min_side_distance >= wall_distance:
            raise ValueError('min_side_distance must be under wall_distance')
        self.linear_speed = linear_speed
        self.turn_speed = turn_speed
        self.turn_radius = abs(turn_radius)
        self.goal_tolerance = goal_tolerance
        self.obstacle_distance = obstacle_distance
        self.clear_margin = clear_margin
        self.wall_distance = wall_distance
        self.min_side_distance = min_side_distance
        self.line_tolerance = line_tolerance
        self.progress_epsilon = progress_epsilon
        self.align_tolerance = math.radians(align_tolerance_deg)
        self.heading_gain = heading_gain
        self.max_yaw_rate = max_yaw_rate
        self.loop_clear_distance = loop_clear_distance
        self.loop_tolerance = loop_tolerance
        self.follow_limit = follow_limit
        self.stuck_timeout = stuck_timeout
        self.stuck_distance = stuck_distance
        self.stuck_angle = math.radians(stuck_angle_deg)
        self.backup_distance = backup_distance
        self.backup_speed = backup_speed

        self.goal = (0.0, 0.0)
        self.start = None
        self.state = Bug2State.GO_TO_GOAL
        self.hit_point = None
        self.hit_distance = math.inf
        self.left_hit_point = False
        self.follow_distance = 0.0
        # Why the vehicle gave up, for whoever has to read the log.
        self.give_up_reason = ''
        self._previous_position = None
        # Progress tracking for the stuck escape: the pose the vehicle was
        # last making progress from, how long it has failed to beat it, and
        # what it was doing before it gave up.
        self._anchor = None
        self._still_time = 0.0
        self._last_command = (0.0, 0.0)
        self._backup_left = 0.0
        self._resume_state = Bug2State.GO_TO_GOAL
        self.set_goal(goal)

    def set_goal(self, goal):
        """Aim at a new goal and forget everything about the old one.

        The m-line is anchored at wherever the vehicle is on the next
        update(), not where it started the run, so a goal assigned after a
        failed one gets a fresh line to follow.
        """
        self.goal = (float(goal[0]), float(goal[1]))
        self.start = None
        self.state = Bug2State.GO_TO_GOAL
        self._forget_hit_point()
        self.give_up_reason = ''
        self._anchor = None
        self._still_time = 0.0
        self._backup_left = 0.0

    @property
    def done(self) -> bool:
        """Whether the vehicle has stopped for good, either way."""
        return self.state in (Bug2State.REACHED, Bug2State.UNREACHABLE)

    def distance_to_goal(self, position) -> float:
        """Straight-line distance from position to the goal, in meters."""
        return math.hypot(self.goal[0] - position[0],
                          self.goal[1] - position[1])

    def update(self, pose, sectors, dt=0.05):
        """Advance the state machine one tick.

        pose is (x, y, yaw) in the odom frame; sectors is
        (right, forward, left) in meters, each the closest beam in that
        sector; dt is the seconds since the last call, which only the stuck
        escape cares about and which defaults to the node's 20 Hz period.
        Returns (linear_x, angular_z) in the body frame.
        """
        self._track_progress(pose, dt)
        self._track_detour(pose)
        command = self._next_command(pose, sectors, dt)
        self._last_command = command
        return command

    def _track_progress(self, pose, dt):
        """Time how long the vehicle has been told to move while it has not.

        Progress is measured against an anchor pose rather than the previous
        tick, because a tick of an honest in-place turn moves the vehicle no
        further than a tick of being stuck against a wall does.
        """
        if self._last_command == (0.0, 0.0) or self.stuck_timeout <= 0.0:
            self._anchor = None
            self._still_time = 0.0
            return
        if self._anchor is None:
            self._anchor = pose
            self._still_time = 0.0
            return
        moved = math.hypot(pose[0] - self._anchor[0], pose[1] - self._anchor[1])
        turned = abs(normalize_angle(pose[2] - self._anchor[2]))
        if moved > self.stuck_distance or turned > self.stuck_angle:
            self._anchor = pose
            self._still_time = 0.0
        else:
            self._still_time += dt

    def _track_detour(self, pose):
        """Add up the distance travelled since the m-line was left.

        Only ground covered off the line counts, which is what follow_limit
        is a budget for.
        """
        position = (pose[0], pose[1])
        if self._previous_position is not None and self.hit_point is not None:
            self.follow_distance += math.hypot(
                position[0] - self._previous_position[0],
                position[1] - self._previous_position[1])
        self._previous_position = position

    def _next_command(self, pose, sectors, dt):
        """Run one tick of the state machine proper. See update()."""
        x, y, yaw = pose
        position = (x, y)
        if self.start is None:
            self.start = position

        if self.done:
            return 0.0, 0.0

        goal_distance = self.distance_to_goal(position)
        if goal_distance <= self.goal_tolerance:
            self.state = Bug2State.REACHED
            return 0.0, 0.0

        if self.state is Bug2State.BACK_UP:
            self._backup_left -= self.backup_speed * dt
            if self._backup_left > 0.0:
                return -self.backup_speed, 0.0
            self.state = self._resume_state
            self._anchor = None
            self._still_time = 0.0
        elif self._still_time >= self.stuck_timeout > 0.0:
            # Told to move, went nowhere: wedged on something the forward
            # sector cannot see. Reverse out and try again from there.
            self._resume_state = self.state
            self.state = Bug2State.BACK_UP
            self._backup_left = self.backup_distance
            self._anchor = None
            self._still_time = 0.0
            return -self.backup_speed, 0.0

        right, forward, _left = sectors
        forward_blocked = forward < self.obstacle_distance
        right_blocked = right < self.wall_distance

        if self.state is Bug2State.GO_TO_GOAL:
            if not forward_blocked:
                return self._steer_toward_goal(pose)
            # Leaving the m-line here; remember the spot so wall following
            # knows what counts as progress and what counts as a full lap.
            self.hit_point = position
            self.hit_distance = goal_distance
            self.left_hit_point = False
            self.state = Bug2State.TURN_CCW

        if self.state is Bug2State.TURN_CCW:
            # Leave on the wider threshold, not the one that got us here:
            # a gap of exactly obstacle_distance is one the first step
            # forward closes again.
            if forward < self.obstacle_distance + self.clear_margin:
                return 0.0, self.turn_speed
            # Front is clear: pick up the wall this same tick.
            self.state = Bug2State.FOLLOW_WALL

        return self._follow_wall(
            position, goal_distance, forward_blocked, right_blocked, right,
            pose)

    def _follow_wall(self, position, goal_distance, forward_blocked,
                     right_blocked, right, pose):
        """One tick of wall following, including the two ways out of it."""
        if self._rejoined_m_line(position, goal_distance):
            self.state = Bug2State.GO_TO_GOAL
            self._forget_hit_point()
            return self._steer_toward_goal(pose)

        if self._circled_back(position):
            return self._give_up(
                'wall following came back around to the hit point without '
                'ever crossing the m-line closer to the goal')
        if 0.0 < self.follow_limit <= self.follow_distance:
            # Not a Bug2 rule, and it can fire on a goal that was reachable
            # given another hundred meters. It is here because the
            # alternative is a vehicle that circles a clump of walls until
            # someone notices.
            return self._give_up(
                f'wall following ran {self.follow_distance:.1f} m, past the '
                f'{self.follow_limit:.1f} m budget, without rejoining the '
                'm-line')

        if forward_blocked:
            # Covers "forward and right both obstructed" and the inside of a
            # corner: back off to turning in place until the front is clear.
            self.state = Bug2State.TURN_CCW
            return 0.0, self.turn_speed
        if right < self.min_side_distance:
            # Too close to turn beside: the corners of the body sweep wider
            # than the sensor can see. Drive on, but bend away from it.
            return self.linear_speed, self.linear_speed / self.turn_radius
        if not right_blocked:
            # The wall to starboard ended. Curve CW to find where it went.
            return self.linear_speed, -self.linear_speed / self.turn_radius
        # Front clear, wall still to starboard: carry on alongside it.
        return self.linear_speed, 0.0

    def _rejoined_m_line(self, position, goal_distance) -> bool:
        """Back on the m-line, and meaningfully closer to the goal for it."""
        if self.hit_point is None:
            return False
        if distance_to_line(position, self.start, self.goal) > \
                self.line_tolerance:
            return False
        return goal_distance < self.hit_distance - self.progress_epsilon

    def _circled_back(self, position) -> bool:
        """Whether wall following has come back around to the hit point."""
        if self.hit_point is None:
            return False
        distance = math.hypot(position[0] - self.hit_point[0],
                              position[1] - self.hit_point[1])
        if distance > self.loop_clear_distance:
            self.left_hit_point = True
            return False
        return self.left_hit_point and distance <= self.loop_tolerance

    def _give_up(self, reason):
        """Stop for good and record why. Returns the command to publish."""
        self.state = Bug2State.UNREACHABLE
        self.give_up_reason = reason
        return 0.0, 0.0

    def _forget_hit_point(self):
        self.hit_point = None
        self.hit_distance = math.inf
        self.left_hit_point = False
        self.follow_distance = 0.0

    def _steer_toward_goal(self, pose):
        """Drive at the goal, turning in place first if badly misaligned.

        On the m-line the bearing to the goal *is* the line's heading, so
        steering at the goal rather than along the line costs nothing there
        and pulls the vehicle back onto the line when it is off it.
        """
        x, y, yaw = pose
        error = normalize_angle(
            math.atan2(self.goal[1] - y, self.goal[0] - x) - yaw)
        if abs(error) > self.align_tolerance:
            return 0.0, math.copysign(self.turn_speed, error)
        return self.linear_speed, clamp(
            self.heading_gain * error, self.max_yaw_rate)


class VehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')

        self.declare_parameter('cmd_vel_topic', '/vehicle/cmd_vel')
        self.declare_parameter('depths_topic', '/vehicle/depths')
        self.declare_parameter('odom_topic', '/vehicle/odom')
        self.declare_parameter('goal_topic', '/vehicle/goal')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 1.0)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('turn_radius', 2.0)
        self.declare_parameter('obstacle_distance', 1.5)
        self.declare_parameter('clear_margin', 0.5)
        self.declare_parameter('wall_distance', 1.5)
        self.declare_parameter('min_side_distance', 0.8)
        self.declare_parameter('line_tolerance', 0.3)
        self.declare_parameter('progress_epsilon', 0.3)
        self.declare_parameter('align_tolerance_deg', 20.0)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('loop_clear_distance', 1.5)
        self.declare_parameter('loop_tolerance', 0.5)
        self.declare_parameter('follow_limit', 60.0)
        self.declare_parameter('stuck_timeout', 2.0)
        self.declare_parameter('stuck_distance', 0.25)
        self.declare_parameter('stuck_angle_deg', 25.0)
        self.declare_parameter('backup_distance', 0.8)
        self.declare_parameter('backup_speed', 0.5)
        self.declare_parameter('forward_half_angle_deg', 20.0)
        self.declare_parameter('side_max_angle_deg', 90.0)
        self.declare_parameter('lidar_fov_deg', 180.0)
        self.declare_parameter('log_period', 2.0)

        # float(): a launch argument like linear_speed:=1 arrives as an int,
        # and both Twist fields and the planner's arithmetic want floats.
        def number(name):
            return float(self.get_parameter(name).value)

        self.lidar_fov_deg = number('lidar_fov_deg')
        self.forward_half_angle_deg = number('forward_half_angle_deg')
        self.side_max_angle_deg = number('side_max_angle_deg')
        if not 0.0 < self.forward_half_angle_deg < self.side_max_angle_deg:
            raise ValueError(
                'need 0 < forward_half_angle_deg < side_max_angle_deg')
        self.log_period = number('log_period')

        self.planner = Bug2Planner(
            (number('goal_x'), number('goal_y')),
            linear_speed=number('linear_speed'),
            turn_speed=number('turn_speed'),
            turn_radius=number('turn_radius'),
            goal_tolerance=number('goal_tolerance'),
            obstacle_distance=number('obstacle_distance'),
            clear_margin=number('clear_margin'),
            wall_distance=number('wall_distance'),
            min_side_distance=number('min_side_distance'),
            line_tolerance=number('line_tolerance'),
            progress_epsilon=number('progress_epsilon'),
            align_tolerance_deg=number('align_tolerance_deg'),
            heading_gain=number('heading_gain'),
            max_yaw_rate=number('max_yaw_rate'),
            loop_clear_distance=number('loop_clear_distance'),
            loop_tolerance=number('loop_tolerance'),
            follow_limit=number('follow_limit'),
            stuck_timeout=number('stuck_timeout'),
            stuck_distance=number('stuck_distance'),
            stuck_angle_deg=number('stuck_angle_deg'),
            backup_distance=number('backup_distance'),
            backup_speed=number('backup_speed'))

        # Latest depths from lidar_scanner, in meters. Empty until the first
        # scan arrives, which is the case every consumer has to handle: the
        # controller starts publishing before Gazebo's sensor is up. Pose is
        # (x, y, yaw) in the odom frame, None until the first odometry.
        self.depths = []
        self.pose = None
        self._last_state = None

        topic = self.get_parameter('cmd_vel_topic').value
        depths_topic = self.get_parameter('depths_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        goal_topic = self.get_parameter('goal_topic').value

        self.pub = self.create_publisher(Twist, topic, 10)
        # Matches the publisher in lidar_scanner: best effort, shallow queue.
        self.create_subscription(
            Float32MultiArray, depths_topic, self._on_depths,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        # A goal is a command, not a sample: reliable, so a single manual
        # `ros2 topic pub` of it cannot be dropped.
        self.create_subscription(Point, goal_topic, self._on_goal, 10)
        # The planner measures the stuck timeout in seconds, and this is
        # the only clock it gets. A tick that runs late only makes the
        # escape trigger later, which is the harmless direction.
        self.period = 1.0 / float(self.get_parameter('rate').value)
        self.create_timer(self.period, self._on_timer)

        goal = self.planner.goal
        self.get_logger().info(
            f'Bug2 to goal ({goal[0]:.2f}, {goal[1]:.2f}), stopping within '
            f'{self.planner.goal_tolerance:.2f} m of it, at '
            f'{self.planner.linear_speed:.2f} m/s, commanding {topic}')
        self.get_logger().info(
            f'Listening for depths on {depths_topic}, odometry on '
            f'{odom_topic}, new goals on {goal_topic}')

    def nearest_obstacle(self):
        """Closest reading in the current scan as (depth_m, bearing_deg).

        Returns None when no scan has arrived yet.
        """
        if not self.depths:
            return None
        index = min(range(len(self.depths)), key=self.depths.__getitem__)
        bearing = beam_bearing_deg(
            index, len(self.depths), self.lidar_fov_deg)
        return self.depths[index], bearing

    def sector_depths(self):
        """Closest depth in each sector as (right, forward, left), in meters.

        Returns None when no scan has arrived yet. Sector edges come from
        forward_half_angle_deg and side_max_angle_deg; see the module
        docstring for why the forward one is narrow.
        """
        if not self.depths:
            return None
        forward_edge = self.forward_half_angle_deg
        side_edge = self.side_max_angle_deg
        return (
            sector_min(self.depths, self.lidar_fov_deg,
                       -side_edge, -forward_edge),
            sector_min(self.depths, self.lidar_fov_deg,
                       -forward_edge, forward_edge),
            sector_min(self.depths, self.lidar_fov_deg,
                       forward_edge, side_edge),
        )

    def compute_command(self) -> Twist:
        """Twist for this tick: Bug2 if it can see and locate itself, else 0.

        Holding still is the only safe answer before the first odometry or
        the first scan. In practice that window is short - the launch file
        starts this node after Gazebo is already publishing both.
        """
        cmd = Twist()
        sectors = self.sector_depths()
        if self.pose is None or sectors is None:
            missing = []
            if self.pose is None:
                missing.append('odometry')
            if sectors is None:
                missing.append('lidar depths')
            self.get_logger().warning(
                f'Waiting for {" and ".join(missing)}; holding still',
                throttle_duration_sec=max(self.log_period, 1.0))
            return cmd
        linear, angular = self.planner.update(self.pose, sectors, self.period)
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self._log_state(sectors)
        return cmd

    def _log_state(self, sectors):
        """Log every state change, plus a periodic line in between."""
        state = self.planner.state
        right, forward, left = sectors
        message = (
            f'{state.value}: {self.planner.distance_to_goal(self.pose):.2f} m '
            f'to goal, sectors R {right:.2f} F {forward:.2f} L {left:.2f} m')
        if state is not self._last_state:
            self._last_state = state
            if state is Bug2State.REACHED:
                self.get_logger().info(f'Goal reached. {message}')
            elif state is Bug2State.BACK_UP:
                self.get_logger().warning(
                    'Commanded to move but going nowhere, so something the '
                    f'lidar cannot see is in the way. Reversing '
                    f'{self.planner.backup_distance:.2f} m. {message}')
            elif state is Bug2State.UNREACHABLE:
                self.get_logger().warning(
                    f'Giving up: {self.planner.give_up_reason}. Stopping; '
                    'publish a new goal on '
                    f'{self.get_parameter("goal_topic").value}. {message}')
            else:
                self.get_logger().info(f'-> {message}')
        elif self.log_period > 0.0:
            nearest = self.nearest_obstacle()
            detail = ''
            if nearest is not None:
                detail = (f', nearest {nearest[0]:.2f} m at '
                          f'{nearest[1]:+.1f} deg')
            self.get_logger().info(
                message + detail, throttle_duration_sec=self.log_period)

    def _on_depths(self, msg: Float32MultiArray):
        self.depths = list(msg.data)

    def _on_odom(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.pose = (
            position.x,
            position.y,
            yaw_from_quaternion(orientation.x, orientation.y,
                                orientation.z, orientation.w),
        )

    def _on_goal(self, msg: Point):
        self.planner.set_goal((msg.x, msg.y))
        self._last_state = None
        self.get_logger().info(f'New goal: ({msg.x:.2f}, {msg.y:.2f})')

    def _on_timer(self):
        self.pub.publish(self.compute_command())

    def stop(self):
        # VelocityControl holds the last command indefinitely, so without
        # this the vehicle keeps driving after the node exits.
        self.pub.publish(Twist())


def main(args=None):
    # Skip rclpy's signal handlers: they shut the context down before we get
    # a chance to publish the stop command. Turn SIGINT and SIGTERM into
    # KeyboardInterrupt instead, which leaves the publisher usable. Set both
    # explicitly, since a process started in the background inherits SIGINT
    # as ignored.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal.default_int_handler)
    node = VehicleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
