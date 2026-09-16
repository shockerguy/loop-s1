import math
import random

from geometry_msgs.msg import Pose
import pytest
import rclpy
from waypoint_nav.random_cube_spawner import (
    footprint_distance_to_origin,
    footprints_overlap,
    RandomCubeSpawner,
)


def test_distance_to_origin():
    # 2 x 1 rectangle centered at (3, 0): near edge at x = 2.
    assert footprint_distance_to_origin(3, 0, 0, 2, 1) == pytest.approx(2.0)
    # Yawed 90 deg, the long side now runs along y: near edge at x = 2.5.
    assert footprint_distance_to_origin(
        3, 0, math.pi / 2, 2, 1) == pytest.approx(2.5)
    assert footprint_distance_to_origin(0.2, 0.1, 0.7, 2, 1) == 0.0


def test_overlap_axis_aligned():
    a = (0, 0, 0, 2, 1)
    assert footprints_overlap(a, (1.5, 0, 0, 2, 1))
    assert not footprints_overlap(a, (2.5, 0, 0, 2, 1))
    # Edge contact is not overlap.
    assert not footprints_overlap(a, (2.0, 0, 0, 2, 1))


def test_overlap_rotated():
    # Axis-aligned bounding boxes overlap, but a 45 deg diamond whose corner
    # points away from the other box's corner stays clear.
    a = (0, 0, 0, 2, 2)
    diamond = (2.3, 2.3, math.pi / 4, 2, 2)
    assert not footprints_overlap(a, diamond)
    assert footprints_overlap(a, (1.5, 1.5, math.pi / 4, 2, 2))
    # Crossed walls: long thin bars through the same center.
    assert footprints_overlap((0, 0, 0, 4, 0.2), (0, 0, math.pi / 2, 4, 0.2))


@pytest.fixture
def spawner():
    rclpy.init()
    node = RandomCubeSpawner()
    node.rng = random.Random(1)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_placements_clear_and_disjoint(spawner):
    spawner.clear_radius = 2.0
    for i in range(60):
        placement = spawner.pick_placement()
        assert placement is not None
        x, y, yaw, sx = placement
        assert all(isinstance(v, float) for v in placement)
        assert -spawner.bound <= x <= spawner.bound
        assert -spawner.bound <= y <= spawner.bound
        assert footprint_distance_to_origin(
            x, y, yaw, sx, spawner.size_y) >= spawner.clear_radius
        fp = (x, y, yaw, sx, spawner.size_y)
        for other in spawner._footprints.values():
            assert not footprints_overlap(fp, other)
        spawner._footprints[f'w{i}'] = fp

    # The values must be accepted by the float64 message fields.
    pose = Pose()
    pose.position.x, pose.position.y = x, y


def test_full_area_gives_up(spawner):
    # Keep-out zone larger than the whole spawn square.
    spawner.clear_radius = 100.0
    assert spawner.pick_placement() is None
