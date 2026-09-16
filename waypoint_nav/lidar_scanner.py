#!/usr/bin/env python3
"""Turn the vehicle's simulated 2D lidar into a plain 1D array of depths.

The sensing itself is Gazebo's built-in GPU lidar: models/vehicle/model.sdf
declares a <sensor type="gpu_lidar"> on the front of the vehicle, the world's
gz::sim::systems::Sensors renders it, and ros_gz_bridge hands the resulting
gz.msgs.LaserScan to ROS 2 as a sensor_msgs/LaserScan. This node is the
consumer of that scan.

A raw scan carries values that arithmetic in a controller cannot use:

  +inf   beam hit nothing within range_max
  -inf   beam hit something inside the sensor's blind spot (below range_min)
  NaN    the renderer produced no reading for that beam at all

This node clamps all of it into a fixed-length array of finite depths in
meters and republishes it as std_msgs/Float32MultiArray for
vehicle_controller, so the consumer only ever sees numbers it can compare.

Array layout
------------
data[0] is the beam at the scan's angle_min and data[-1] the beam at
angle_max. The sensor faces +x (vehicle forward) with a symmetric field of
view, so index 0 looks to starboard (the vehicle's right), the middle index
looks straight ahead, and the last index looks to port. The length equals
<samples> in the SDF and does not change between scans.

Parameters
----------
scan_topic : str    LaserScan topic from the bridge. Default "/vehicle/scan".
depths_topic : str  Float32MultiArray topic to publish. Default "/vehicle/depths".
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def sanitize_ranges(ranges, range_min: float, range_max: float):
    """Clamp raw LaserScan ranges to finite depths in [range_min, range_max].

    Returns (depths, dropouts), where dropouts counts the NaN beams.

    A beam that hit nothing reads +inf and becomes range_max: nothing is out
    there, as far as the sensor can tell. A beam below range_min reads -inf
    or a near-zero and becomes range_min: something is right on top of us.

    NaN also becomes range_max, which is the optimistic reading. Treating a
    renderer dropout as an obstacle would let one bad frame stop the vehicle,
    but it does mean a consumer cannot distinguish "clear" from "no data" -
    hence the returned count, which the node logs.
    """
    depths = []
    dropouts = 0
    for r in ranges:
        if math.isnan(r):
            dropouts += 1
            depths.append(range_max)
        elif r >= range_max:  # also catches +inf
            depths.append(range_max)
        elif r <= range_min:  # also catches -inf and 0.0
            depths.append(range_min)
        else:
            depths.append(r)
    return depths, dropouts


class LidarScanner(Node):

    def __init__(self):
        super().__init__('lidar_scanner')

        self.declare_parameter('scan_topic', '/vehicle/scan')
        self.declare_parameter('depths_topic', '/vehicle/depths')

        scan_topic = self.get_parameter('scan_topic').value
        depths_topic = self.get_parameter('depths_topic').value

        # Sensor-data QoS (best effort, shallow queue): a stale scan is worth
        # less than the next one, so dropping beats blocking. The bridge
        # publishes reliably, which still satisfies a best-effort subscriber.
        self.pub = self.create_publisher(
            Float32MultiArray, depths_topic, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)

        # The scan geometry is fixed by the SDF, so log it once rather than
        # every frame. None until the first scan arrives.
        self._geometry = None

        self.get_logger().info(
            f'Relaying depths from {scan_topic} to {depths_topic}')

    def _log_geometry_once(self, msg: LaserScan):
        geometry = (len(msg.ranges), msg.angle_min, msg.angle_max,
                    msg.range_min, msg.range_max)
        if geometry == self._geometry:
            return
        self._geometry = geometry
        self.get_logger().info(
            f'Scan geometry: {len(msg.ranges)} beams over '
            f'[{math.degrees(msg.angle_min):.1f}, '
            f'{math.degrees(msg.angle_max):.1f}] deg, range '
            f'[{msg.range_min:.2f}, {msg.range_max:.2f}] m, '
            f'frame {msg.header.frame_id!r}')

    def _on_scan(self, msg: LaserScan):
        if not msg.ranges:
            # Gazebo can publish an empty scan on the first tick, before the
            # sensor has rendered anything.
            return
        if not math.isfinite(msg.range_max) or msg.range_max <= msg.range_min:
            self.get_logger().warning(
                f'Ignoring scan with unusable range bounds '
                f'[{msg.range_min}, {msg.range_max}]',
                throttle_duration_sec=5.0)
            return

        self._log_geometry_once(msg)

        depths, dropouts = sanitize_ranges(
            msg.ranges, msg.range_min, msg.range_max)
        if dropouts:
            self.get_logger().warning(
                f'{dropouts}/{len(depths)} beams had no reading; reporting '
                f'them as clear at {msg.range_max:.2f} m',
                throttle_duration_sec=5.0)

        self.pub.publish(self.build_message(depths))

    @staticmethod
    def build_message(depths) -> Float32MultiArray:
        out = Float32MultiArray()
        # A flat array needs no layout to be readable, but filling it in means
        # `ros2 topic echo` and any generic consumer can tell what the axis is.
        dim = MultiArrayDimension()
        dim.label = 'beam'
        dim.size = len(depths)
        dim.stride = len(depths)
        out.layout.dim = [dim]
        out.layout.data_offset = 0
        out.data = [float(d) for d in depths]
        return out


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanner()
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
