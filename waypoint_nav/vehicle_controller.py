#!/usr/bin/env python3
"""Drive the vehicle by publishing body-frame velocity commands.

The default behavior is an open-loop circle: constant forward speed and a
yaw rate of linear_speed / turn_radius. Positive turn_radius turns left
(counter-clockwise seen from above), negative turns right. The vehicle
starts at the origin facing +x, so the circle is centered on (0, turn_radius).

The node also subscribes to the forward lidar depths published by
lidar_scanner: a std_msgs/Float32MultiArray of finite ranges in meters,
index 0 at the starboard edge of the field of view and the last index at the
port edge. Nothing steers off them yet - apply_lidar() is a placeholder hook
where obstacle handling goes.

Replace compute_command() to give the vehicle a different open-loop behavior,
or fill in apply_lidar() to make it react to what the lidar sees.

Parameters
----------
cmd_vel_topic : str   Twist topic the vehicle listens on. Default "/vehicle/cmd_vel".
depths_topic : str    Depth array from lidar_scanner. Default "/vehicle/depths".
rate : float          Publish rate in Hz. Default 20.0.
linear_speed : float  Forward speed in m/s. Default 1.0.
turn_radius : float   Circle radius in meters. Default 3.0.
lidar_fov_deg : float Total field of view of the depth array, used to turn an
                      index into a bearing. Must match <min_angle>/<max_angle>
                      in models/vehicle/model.sdf. Default 180.0.
log_period : float    Seconds between nearest-obstacle log lines; 0 disables
                      them. Default 2.0.
"""

import signal

import rclpy
from geometry_msgs.msg import Twist
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


class VehicleController(Node):

    def __init__(self):
        super().__init__('vehicle_controller')

        self.declare_parameter('cmd_vel_topic', '/vehicle/cmd_vel')
        self.declare_parameter('depths_topic', '/vehicle/depths')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('turn_radius', 3.0)
        self.declare_parameter('lidar_fov_deg', 180.0)
        self.declare_parameter('log_period', 2.0)

        # float(): a launch argument like linear_speed:=1 arrives as an int,
        # and Twist fields reject ints.
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.turn_radius = float(self.get_parameter('turn_radius').value)
        if self.turn_radius == 0.0:
            raise ValueError('turn_radius must be nonzero')
        self.lidar_fov_deg = float(self.get_parameter('lidar_fov_deg').value)
        self.log_period = float(self.get_parameter('log_period').value)

        # Latest depths from lidar_scanner, in meters. Empty until the first
        # scan arrives, which is the case every consumer has to handle: the
        # controller starts publishing before Gazebo's sensor is up.
        self.depths = []

        topic = self.get_parameter('cmd_vel_topic').value
        depths_topic = self.get_parameter('depths_topic').value

        self.pub = self.create_publisher(Twist, topic, 10)
        # Matches the publisher in lidar_scanner: best effort, shallow queue.
        self.create_subscription(
            Float32MultiArray, depths_topic, self._on_depths,
            qos_profile_sensor_data)
        self.create_timer(
            1.0 / self.get_parameter('rate').value, self._on_timer)

        self.get_logger().info(
            f'Driving in a circle: {self.linear_speed:.2f} m/s, '
            f'radius {self.turn_radius:.2f} m, on {topic}')
        self.get_logger().info(f'Listening for lidar depths on {depths_topic}')

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

    def apply_lidar(self, cmd: Twist) -> Twist:
        """Adjust cmd using self.depths. Placeholder; currently a no-op.

        Real handling goes here. self.depths is a list of finite ranges in
        meters spanning lidar_fov_deg, starboard to port, already cleaned of
        inf and NaN by lidar_scanner, so it is safe to call min() or slice a
        sector out of it directly. beam_bearing_deg() maps an index back to a
        bearing, and nearest_obstacle() is the common case of both.

        For example, a stop-on-obstacle rule would be::

            nearest = self.nearest_obstacle()
            if nearest is not None and nearest[0] < self.stop_distance:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        Until something like that is filled in, the vehicle keeps circling
        regardless of what the lidar sees.
        """
        if not self.depths:
            # No scan yet: drive open loop.
            return cmd
        # TODO: react to self.depths here.
        return cmd

    def compute_command(self) -> Twist:
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.linear_speed / self.turn_radius
        return self.apply_lidar(cmd)

    def _on_depths(self, msg: Float32MultiArray):
        self.depths = list(msg.data)
        if self.log_period <= 0.0:
            return
        nearest = self.nearest_obstacle()
        if nearest is None:
            return
        depth, bearing = nearest
        self.get_logger().info(
            f'Nearest of {len(self.depths)} beams: {depth:.2f} m at '
            f'{bearing:+.1f} deg',
            throttle_duration_sec=self.log_period)

    def _on_timer(self):
        self.pub.publish(self.compute_command())

    def stop(self):
        # VelocityControl holds the last command indefinitely, so without
        # this the vehicle keeps circling after the node exits.
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
