import math
import random

import pytest
from waypoint_nav.random_cube_spawner import walls_from_msg, walls_to_msg
from waypoint_nav.waypoint_generator import (
    distance_to_wall,
    pick_waypoint,
    waypoint_is_valid,
)


def test_distance_to_wall():
    wall = (5.0, 5.0, 0.0, 2.0, 1.0)
    # Near long edge at y = 4.5.
    assert distance_to_wall((5.0, 3.0), wall) == pytest.approx(1.5)
    # Near short edge at x = 6.
    assert distance_to_wall((7.0, 5.0), wall) == pytest.approx(1.0)
    assert distance_to_wall((5.2, 5.1), wall) == 0.0
    # Yawed 90 deg the long side runs along y, so the short edge is at y = 4.
    turned = (5.0, 5.0, math.pi / 2, 2.0, 1.0)
    assert distance_to_wall((5.0, 3.0), turned) == pytest.approx(1.0)


def test_waypoint_is_valid():
    walls = [(0.0, 0.0, 0.0, 2.0, 1.0)]
    assert waypoint_is_valid((0.0, 2.0), walls, 1.5, 12.0)
    assert not waypoint_is_valid((0.0, 1.9), walls, 1.5, 12.0)
    assert not waypoint_is_valid((12.5, 0.0), walls, 1.5, 12.0)
    assert not waypoint_is_valid(
        (0.0, 5.0), walls, 1.5, 12.0, vehicle=(0.0, 4.0), min_travel=2.0)
    assert waypoint_is_valid(
        (0.0, 5.0), walls, 1.5, 12.0, vehicle=(0.0, 0.0), min_travel=2.0)


def test_picked_waypoints_obey_the_rules():
    rng = random.Random(3)
    walls = [(rng.uniform(-10, 10), rng.uniform(-10, 10),
              rng.uniform(-math.pi, math.pi), rng.uniform(1, 4), 1.0)
             for _ in range(12)]
    vehicle = (0.0, 0.0)
    for _ in range(200):
        point = pick_waypoint(rng, walls, 1.5, 12.0, vehicle, 2.0)
        assert point is not None
        assert all(-12.0 <= v <= 12.0 for v in point)
        assert min(distance_to_wall(point, w) for w in walls) >= 1.5
        assert math.dist(point, vehicle) >= 2.0
        vehicle = point


def test_no_room_returns_none():
    # One wall covering the whole area.
    walls = [(0.0, 0.0, 0.0, 30.0, 30.0)]
    assert pick_waypoint(random.Random(0), walls, 1.5, 12.0) is None


def test_walls_round_trip():
    walls = [(1.0, -2.0, 0.5, 3.0, 1.0), (4.0, 5.0, -1.0, 1.5, 1.0)]
    msg = walls_to_msg(walls)
    assert [d.size for d in msg.layout.dim] == [2, 5]
    assert walls_from_msg(msg) == walls
    assert walls_from_msg(walls_to_msg([])) == []
