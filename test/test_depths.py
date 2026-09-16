import math

import pytest
import rclpy
from std_msgs.msg import Float32MultiArray
from waypoint_nav.lidar_scanner import LidarScanner, sanitize_ranges
from waypoint_nav.vehicle_controller import beam_bearing_deg, VehicleController

# Matches the front_lidar sensor in models/vehicle/model.sdf.
RANGE_MIN, RANGE_MAX = 0.1, 10.0
BEAMS, FOV_DEG = 181, 180.0


def test_sanitize_passes_through_valid_readings():
    depths, dropouts = sanitize_ranges([0.5, 3.25, 9.9], RANGE_MIN, RANGE_MAX)
    assert depths == pytest.approx([0.5, 3.25, 9.9])
    assert dropouts == 0


def test_sanitize_clamps_out_of_range():
    raw = [math.inf, -math.inf, 0.0, 20.0, -5.0, RANGE_MAX, RANGE_MIN]
    depths, dropouts = sanitize_ranges(raw, RANGE_MIN, RANGE_MAX)
    # No hit and beyond max both mean "clear"; anything at or under the blind
    # spot means "right on top of us".
    assert depths == pytest.approx(
        [RANGE_MAX, RANGE_MIN, RANGE_MIN, RANGE_MAX, RANGE_MIN,
         RANGE_MAX, RANGE_MIN])
    assert dropouts == 0


def test_sanitize_counts_nan_and_reports_it_as_clear():
    depths, dropouts = sanitize_ranges(
        [1.0, math.nan, math.nan, 2.0], RANGE_MIN, RANGE_MAX)
    assert depths == pytest.approx([1.0, RANGE_MAX, RANGE_MAX, 2.0])
    assert dropouts == 2


def test_sanitize_output_is_always_finite_and_in_bounds():
    raw = [math.inf, -math.inf, math.nan, 0.0, 1e9, -1e9, 4.2]
    depths, _ = sanitize_ranges(raw, RANGE_MIN, RANGE_MAX)
    assert len(depths) == len(raw)
    assert all(math.isfinite(d) for d in depths)
    assert all(RANGE_MIN <= d <= RANGE_MAX for d in depths)


def test_build_message_is_a_flat_labeled_array():
    msg = LidarScanner.build_message([1.0, 2.0, 3.0])
    assert isinstance(msg, Float32MultiArray)
    assert list(msg.data) == pytest.approx([1.0, 2.0, 3.0])
    assert len(msg.layout.dim) == 1
    assert msg.layout.dim[0].size == 3
    assert msg.layout.dim[0].stride == 3
    assert msg.layout.data_offset == 0


def test_build_message_accepts_ints():
    # float64 message fields reject Python ints, so the node must convert.
    msg = LidarScanner.build_message([1, 2, 3])
    assert list(msg.data) == pytest.approx([1.0, 2.0, 3.0])


def test_bearing_spans_the_fan_starboard_to_port():
    assert beam_bearing_deg(0, BEAMS, FOV_DEG) == pytest.approx(-90.0)
    assert beam_bearing_deg(BEAMS // 2, BEAMS, FOV_DEG) == pytest.approx(0.0)
    assert beam_bearing_deg(BEAMS - 1, BEAMS, FOV_DEG) == pytest.approx(90.0)
    # 181 beams over 180 deg is one per degree.
    assert beam_bearing_deg(1, BEAMS, FOV_DEG) == pytest.approx(-89.0)


def test_bearing_of_a_degenerate_fan():
    assert beam_bearing_deg(0, 1, FOV_DEG) == 0.0
    assert beam_bearing_deg(0, 0, FOV_DEG) == 0.0


@pytest.fixture
def controller():
    rclpy.init()
    node = VehicleController()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_controller_has_no_depths_before_the_first_scan(controller):
    assert controller.depths == []
    assert controller.nearest_obstacle() is None


def test_controller_finds_the_nearest_beam(controller):
    depths = [RANGE_MAX] * BEAMS
    depths[45] = 2.5   # 45 deg to starboard
    depths[100] = 3.0
    controller._on_depths(LidarScanner.build_message(depths))

    assert len(controller.depths) == BEAMS
    depth, bearing = controller.nearest_obstacle()
    assert depth == pytest.approx(2.5)
    assert bearing == pytest.approx(-45.0)


def test_placeholder_leaves_the_open_loop_command_alone(controller):
    baseline = controller.compute_command()

    depths = [RANGE_MIN] * BEAMS  # a wall pressed against the sensor
    controller._on_depths(LidarScanner.build_message(depths))
    cmd = controller.compute_command()

    # apply_lidar() is still a stub, so the circle is unchanged. This test is
    # the tripwire: filling in real handling should make it fail.
    assert cmd.linear.x == baseline.linear.x == controller.linear_speed
    assert cmd.angular.z == pytest.approx(baseline.angular.z)
