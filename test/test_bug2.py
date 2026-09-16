"""Unit tests for the Bug2 planner, plus closed-loop runs against fake walls.

The planner takes a pose and three sector depths and returns a body-frame
velocity, so a few dozen lines of ray casting and Euler integration are
enough to fly the whole algorithm around an obstacle course without Gazebo.
"""

import math

import pytest
from waypoint_nav.vehicle_controller import (
    beam_bearing_deg,
    Bug2Planner,
    Bug2State,
    clamp,
    distance_to_line,
    normalize_angle,
    sector_min,
    yaw_from_quaternion,
)

FOV_DEG = 180.0
# 5 deg apart. The real sensor has 181 beams; the sectors are what the
# planner sees, and they resolve fine at this density.
SIM_BEAMS = 37
RANGE_MAX = 10.0
FORWARD_HALF, SIDE_MAX = 20.0, 90.0


# --- helpers ---------------------------------------------------------------

def test_normalize_angle_wraps_into_half_turns():
    assert normalize_angle(0.0) == pytest.approx(0.0)
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert abs(normalize_angle(100.0)) <= math.pi


def test_yaw_from_quaternion_recovers_the_angle():
    for yaw in (0.0, 0.7, -2.5, math.pi / 2):
        half = yaw / 2.0
        assert yaw_from_quaternion(
            0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(yaw)


def test_distance_to_line_is_perpendicular():
    start, goal = (0.0, 0.0), (10.0, 0.0)
    assert distance_to_line((5.0, 0.0), start, goal) == pytest.approx(0.0)
    assert distance_to_line((5.0, 2.0), start, goal) == pytest.approx(2.0)
    # Off the end of the segment still counts: the m-line test is about the
    # infinite line, and progress toward the goal is checked separately.
    assert distance_to_line((20.0, -3.0), start, goal) == pytest.approx(3.0)
    diagonal = distance_to_line((1.0, 0.0), (0.0, 0.0), (1.0, 1.0))
    assert diagonal == pytest.approx(math.sqrt(0.5))


def test_distance_to_a_degenerate_line_is_distance_to_the_point():
    assert distance_to_line(
        (3.0, 4.0), (0.0, 0.0), (0.0, 0.0)) == pytest.approx(5.0)


def test_clamp_limits_both_signs():
    assert clamp(5.0, 1.0) == 1.0
    assert clamp(-5.0, 1.0) == -1.0
    assert clamp(0.4, 1.0) == pytest.approx(0.4)


def test_sector_min_picks_the_closest_beam_in_the_sector():
    depths = [RANGE_MAX] * SIM_BEAMS
    depths[0] = 1.0     # -90 deg, starboard edge
    depths[18] = 2.0    # straight ahead
    depths[36] = 0.5    # +90 deg, port edge
    assert sector_min(depths, FOV_DEG, -90, -20) == pytest.approx(1.0)
    assert sector_min(depths, FOV_DEG, -20, 20) == pytest.approx(2.0)
    assert sector_min(depths, FOV_DEG, 20, 90) == pytest.approx(0.5)


def test_sector_min_of_an_empty_sector_is_infinite():
    assert sector_min([], FOV_DEG, -20, 20) == math.inf
    # No beam lands inside half a degree of straight ahead at 5 deg spacing.
    assert sector_min([RANGE_MAX] * 36, FOV_DEG, -0.1, 0.1) == math.inf


# --- planner ---------------------------------------------------------------

def make_planner(goal=(8.0, 0.0), **kwargs):
    return Bug2Planner(goal, **kwargs)


CLEAR = (RANGE_MAX, RANGE_MAX, RANGE_MAX)


def test_drives_straight_at_a_goal_dead_ahead():
    planner = make_planner()
    v, w = planner.update((0.0, 0.0, 0.0), CLEAR)
    assert v == pytest.approx(1.0)
    assert w == pytest.approx(0.0)
    assert planner.state is Bug2State.GO_TO_GOAL
    # The m-line is anchored wherever the first update found the vehicle.
    assert planner.start == (0.0, 0.0)


def test_turns_in_place_when_badly_misaligned():
    planner = make_planner()
    v, w = planner.update((0.0, 0.0, math.pi), CLEAR)
    assert v == 0.0
    assert abs(w) == pytest.approx(planner.turn_speed)
    # Goal to starboard of a reversed heading: turn starboard, not port.
    v, w = planner.update((0.0, 0.0, math.pi / 2 + 0.5), CLEAR)
    assert w < 0.0


def test_steers_proportionally_when_nearly_aligned():
    planner = make_planner(heading_gain=2.0)
    v, w = planner.update((0.0, 0.0, -0.1), CLEAR)
    assert v == pytest.approx(planner.linear_speed)
    assert w == pytest.approx(0.2)


def test_yaw_rate_is_capped():
    planner = make_planner(heading_gain=100.0, max_yaw_rate=0.4)
    _, w = planner.update((0.0, 0.0, -0.3), CLEAR)
    assert w == pytest.approx(0.4)


def test_stops_inside_the_goal_tolerance():
    planner = make_planner(goal_tolerance=1.0)
    v, w = planner.update((7.5, 0.0, 0.0), CLEAR)
    assert (v, w) == (0.0, 0.0)
    assert planner.state is Bug2State.REACHED
    assert planner.done
    # Nothing restarts it: a reached goal stays reached.
    assert planner.update((0.0, 0.0, 0.0), CLEAR) == (0.0, 0.0)


def test_a_blocked_front_starts_a_ccw_turn_and_records_the_hit_point():
    planner = make_planner(obstacle_distance=1.5)
    v, w = planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    assert v == 0.0
    assert w == pytest.approx(planner.turn_speed)  # positive is CCW
    assert planner.state is Bug2State.TURN_CCW
    assert planner.hit_point == (2.0, 0.0)
    assert planner.hit_distance == pytest.approx(6.0)


def test_ccw_turn_ends_when_the_front_clears_and_wall_following_starts():
    planner = make_planner()
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    # Front clear, wall now to starboard: drive alongside it. This is (X).
    v, w = planner.update((2.0, 0.0, math.pi / 2), (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.state is Bug2State.FOLLOW_WALL
    assert v == pytest.approx(planner.linear_speed)
    assert w == pytest.approx(0.0)


def test_the_ccw_turn_holds_until_the_front_clears_by_the_margin():
    planner = make_planner(obstacle_distance=1.5, clear_margin=0.5)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))

    # Past obstacle_distance but inside the margin: keep turning. Handing
    # back here is what makes the vehicle chatter against a wall.
    v, w = planner.update((2.0, 0.0, 0.3), (RANGE_MAX, 1.7, RANGE_MAX))
    assert planner.state is Bug2State.TURN_CCW
    assert (v, w) == (0.0, pytest.approx(planner.turn_speed))

    planner.update((2.0, 0.0, 0.6), (RANGE_MAX, 2.1, RANGE_MAX))
    assert planner.state is Bug2State.FOLLOW_WALL


def test_a_wall_too_close_to_starboard_is_bent_away_from():
    # A 2 x 1 m body sweeps 1.118 m of corner when it turns in place, so a
    # wall inside min_side_distance is one it would catch on.
    planner = make_planner(min_side_distance=0.8, turn_radius=2.0)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((2.0, 0.0, math.pi / 2), (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.state is Bug2State.FOLLOW_WALL

    v, w = planner.update((2.0, 1.0, math.pi / 2), (0.5, RANGE_MAX, RANGE_MAX))
    assert v == pytest.approx(planner.linear_speed)
    assert w == pytest.approx(0.5)   # CCW, away from the wall, still driving
    assert planner.state is Bug2State.FOLLOW_WALL


def test_a_side_clearance_wider_than_the_wall_distance_is_rejected():
    with pytest.raises(ValueError):
        make_planner(min_side_distance=1.5, wall_distance=1.5)


def test_a_wall_that_ends_is_chased_with_a_cw_arc():
    planner = make_planner(turn_radius=2.0)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((2.0, 0.0, math.pi / 2), (1.0, RANGE_MAX, RANGE_MAX))
    v, w = planner.update((2.0, 1.0, math.pi / 2), CLEAR)
    assert planner.state is Bug2State.FOLLOW_WALL
    assert v == pytest.approx(planner.linear_speed)
    assert w == pytest.approx(-0.5)  # CW, radius 2 m at 1 m/s


def test_a_corner_with_both_sectors_blocked_turns_ccw_again():
    planner = make_planner()
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((2.0, 0.0, math.pi / 2), (1.0, RANGE_MAX, RANGE_MAX))
    v, w = planner.update((2.0, 1.0, math.pi / 2), (1.0, 1.0, RANGE_MAX))
    assert planner.state is Bug2State.TURN_CCW
    assert (v, w) == (0.0, pytest.approx(planner.turn_speed))


def test_the_m_line_is_only_rejoined_closer_to_the_goal():
    planner = make_planner(progress_epsilon=0.3)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((2.0, 0.0, math.pi / 2), (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.state is Bug2State.FOLLOW_WALL

    # Back on the line but no better off than at the hit point: keep going.
    planner.update((1.9, 0.0, math.pi / 2), (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.state is Bug2State.FOLLOW_WALL
    # Still on the line, now 2 m closer to the goal: take it.
    planner.update((4.0, 0.0, 0.0), (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.state is Bug2State.GO_TO_GOAL
    assert planner.hit_point is None


def test_off_the_line_is_not_a_rejoin_however_close_the_goal_gets():
    planner = make_planner(line_tolerance=0.3)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((2.0, 0.0, math.pi / 2), (1.0, RANGE_MAX, RANGE_MAX))
    planner.update((6.0, 1.5, 0.0), (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.state is Bug2State.FOLLOW_WALL


def test_circling_back_to_the_hit_point_means_the_goal_is_unreachable():
    planner = make_planner(loop_clear_distance=1.5, loop_tolerance=0.5)
    wall_on_the_right = (1.0, RANGE_MAX, RANGE_MAX)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((2.0, 0.0, math.pi / 2), wall_on_the_right)

    # A lap that stays off the m-line and never gets closer to the goal.
    for y in (1.0, 2.0, 3.0, 2.0, 1.0):
        planner.update((0.0, y, 0.0), wall_on_the_right)
        assert planner.state is Bug2State.FOLLOW_WALL
    v, w = planner.update((2.0, 0.2, 0.0), wall_on_the_right)
    assert planner.state is Bug2State.UNREACHABLE
    assert (v, w) == (0.0, 0.0)
    assert planner.done


def test_a_reason_is_recorded_for_giving_up():
    planner = make_planner(loop_clear_distance=1.5, loop_tolerance=0.5)
    wall_on_the_right = (1.0, RANGE_MAX, RANGE_MAX)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((2.0, 0.0, math.pi / 2), wall_on_the_right)
    for y in (1.0, 2.0, 3.0, 2.0, 1.0):
        planner.update((0.0, y, 0.0), wall_on_the_right)
    planner.update((2.0, 0.2, 0.0), wall_on_the_right)
    assert planner.state is Bug2State.UNREACHABLE
    assert 'hit point' in planner.give_up_reason


def test_wall_following_runs_out_of_budget():
    # Bug2 terminates only while it follows one obstacle. A circuit around a
    # clump of them never passes the hit point, and nothing else would stop.
    planner = make_planner(follow_limit=20.0, loop_clear_distance=1e9)
    wall_on_the_right = (1.0, RANGE_MAX, RANGE_MAX)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((2.0, 0.0, math.pi / 2), wall_on_the_right)

    # 30 m of travel, all of it off the m-line and none of it closer to the
    # goal at (8, 0).
    for step in range(1, 31):
        planner.update((2.0, float(step), math.pi / 2), wall_on_the_right)
    assert planner.state is Bug2State.UNREACHABLE
    assert planner.follow_distance >= 20.0
    assert 'budget' in planner.give_up_reason


def test_the_budget_can_be_switched_off():
    planner = make_planner(follow_limit=0.0, loop_clear_distance=1e9)
    wall_on_the_right = (1.0, RANGE_MAX, RANGE_MAX)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((2.0, 0.0, math.pi / 2), wall_on_the_right)
    for step in range(1, 201):
        planner.update((2.0, float(step), math.pi / 2), wall_on_the_right)
    assert planner.state is Bug2State.FOLLOW_WALL


def test_the_budget_only_counts_ground_covered_off_the_m_line():
    planner = make_planner(goal=(12.0, 0.0), follow_limit=20.0)
    # On the m-line: nothing is being spent.
    for step in range(5):
        planner.update((float(step), 0.0, 0.0), CLEAR)
    assert planner.follow_distance == 0.0

    planner.update((5.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    planner.update((5.0, 0.0, math.pi / 2), (1.0, RANGE_MAX, RANGE_MAX))
    planner.update((5.0, 2.0, math.pi / 2), (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.follow_distance == pytest.approx(2.0)

    # Rejoining closer to the goal hands the whole budget back.
    planner.update((7.0, 0.0, 0.0), (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.state is Bug2State.GO_TO_GOAL
    assert planner.follow_distance == 0.0


def test_sitting_near_the_hit_point_is_not_a_lap():
    planner = make_planner(loop_clear_distance=1.5)
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    for _ in range(20):
        planner.update((2.1, 0.1, 1.0), (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.state is Bug2State.FOLLOW_WALL


def test_a_new_goal_restarts_the_run_from_where_the_vehicle_is():
    planner = make_planner()
    planner.update((2.0, 0.0, 0.0), (RANGE_MAX, 1.0, RANGE_MAX))
    assert planner.state is Bug2State.TURN_CCW

    planner.set_goal((0.0, -5.0))
    assert planner.goal == (0.0, -5.0)
    assert planner.state is Bug2State.GO_TO_GOAL
    assert planner.hit_point is None
    assert planner.start is None

    planner.update((3.0, 1.0, 0.0), CLEAR)
    assert planner.start == (3.0, 1.0)  # new m-line, anchored here


def test_a_zero_turn_radius_is_rejected():
    with pytest.raises(ValueError):
        make_planner(turn_radius=0.0)


# --- the stuck escape ------------------------------------------------------

def test_going_nowhere_while_told_to_move_ends_in_a_reverse():
    planner = make_planner(stuck_timeout=2.0, backup_speed=0.5)
    stuck = (0.0, 0.0, 0.0)  # driving at a clear goal, but pinned on it
    ticks = 0
    while planner.state is not Bug2State.BACK_UP:
        v, w = planner.update(stuck, CLEAR)
        ticks += 1
        assert ticks < 100, 'never gave up on a command going nowhere'
    # 2 s of no progress at the default 0.05 s tick, give or take the two
    # ticks it takes to notice a command was issued and drop an anchor.
    assert 40 <= ticks <= 43
    assert v == pytest.approx(-0.5)
    assert w == 0.0


def test_the_reverse_lasts_a_fixed_distance_and_hands_back():
    planner = make_planner(
        stuck_timeout=2.0, backup_distance=0.8, backup_speed=0.5)
    stuck = (2.0, 0.0, 0.0)
    planner.update(stuck, (RANGE_MAX, 1.0, RANGE_MAX))  # wall following
    planner.update(stuck, (1.0, RANGE_MAX, RANGE_MAX))
    assert planner.state is Bug2State.FOLLOW_WALL
    while planner.state is not Bug2State.BACK_UP:
        planner.update(stuck, (1.0, RANGE_MAX, RANGE_MAX))

    # 0.8 m at 0.5 m/s is 1.6 s, or 32 ticks.
    for _ in range(31):
        v, _ = planner.update(stuck, (1.0, RANGE_MAX, RANGE_MAX))
        assert v == pytest.approx(-0.5)
    planner.update(stuck, (1.0, RANGE_MAX, RANGE_MAX))
    # It goes back to what it was doing, not to the start of the algorithm.
    assert planner.state is Bug2State.FOLLOW_WALL


def test_a_vehicle_that_is_moving_never_reverses():
    planner = make_planner(stuck_timeout=2.0)
    for step in range(200):
        planner.update((step * 0.05, 0.0, 0.0), CLEAR)
        assert planner.state is not Bug2State.BACK_UP


def test_turning_in_place_is_not_being_stuck():
    # The whole point of measuring against an anchor: one tick of an honest
    # in-place turn displaces the vehicle no more than being wedged does.
    planner = make_planner(goal=(-5.0, 0.0), stuck_timeout=2.0)
    for step in range(200):
        yaw = normalize_angle(step * 0.6 * 0.05)
        v, w = planner.update((0.0, 0.0, yaw), CLEAR)
        assert planner.state is not Bug2State.BACK_UP


def test_the_escape_can_be_switched_off():
    planner = make_planner(stuck_timeout=0.0)
    for _ in range(200):
        v, _ = planner.update((0.0, 0.0, 0.0), CLEAR)
        assert v > 0.0
    assert planner.state is Bug2State.GO_TO_GOAL


def test_a_new_goal_clears_a_pending_reverse():
    planner = make_planner(stuck_timeout=2.0)
    while planner.state is not Bug2State.BACK_UP:
        planner.update((0.0, 0.0, 0.0), CLEAR)
    planner.set_goal((1.0, 1.0))
    assert planner.state is Bug2State.GO_TO_GOAL
    v, _ = planner.update((0.0, 0.0, math.pi / 4), CLEAR)
    assert v > 0.0


# --- closed loop against fake walls ----------------------------------------

def ray_depth(origin, angle, box):
    """Range from origin along angle to an axis-aligned box, or inf.

    box is (xmin, xmax, ymin, ymax). Slab method: the ray is inside the box
    between the entry and exit times of both axis slabs.
    """
    tmin, tmax = 0.0, math.inf
    for o, d, lo, hi in ((origin[0], math.cos(angle), box[0], box[1]),
                         (origin[1], math.sin(angle), box[2], box[3])):
        if abs(d) < 1e-12:
            if not lo <= o <= hi:
                return math.inf
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        tmin = max(tmin, min(t1, t2))
        tmax = min(tmax, max(t1, t2))
    return tmin if tmin <= tmax else math.inf


def scan(pose, boxes):
    """Cast a fake lidar fan: SIM_BEAMS depths, starboard to port."""
    x, y, yaw = pose
    depths = []
    for index in range(SIM_BEAMS):
        angle = yaw + math.radians(
            beam_bearing_deg(index, SIM_BEAMS, FOV_DEG))
        hit = min((ray_depth((x, y), angle, box) for box in boxes),
                  default=math.inf)
        depths.append(min(hit, RANGE_MAX))
    return depths


def sectors(depths):
    return (sector_min(depths, FOV_DEG, -SIDE_MAX, -FORWARD_HALF),
            sector_min(depths, FOV_DEG, -FORWARD_HALF, FORWARD_HALF),
            sector_min(depths, FOV_DEG, FORWARD_HALF, SIDE_MAX))


def drive(planner, boxes, pose=(0.0, 0.0, 0.0), dt=0.05, steps=6000):
    """Run the planner in closed loop over unicycle kinematics.

    Returns (final_pose, path, closest_approach). The vehicle is a point
    here, so closest_approach is how near its center came to a wall.
    """
    path = [pose]
    closest = math.inf
    for _ in range(steps):
        depths = scan(pose, boxes)
        closest = min(closest, min(depths))
        v, w = planner.update(pose, sectors(depths))
        x, y, yaw = pose
        yaw = normalize_angle(yaw + w * dt)
        pose = (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, yaw)
        path.append(pose)
        if planner.done:
            break
    return pose, path, closest


def test_an_empty_world_is_a_straight_shot():
    planner = make_planner((6.0, 0.0))
    pose, path, _ = drive(planner, [], steps=400)
    assert planner.state is Bug2State.REACHED
    assert max(abs(y) for _, y, _ in path) < 0.1


def test_a_goal_behind_the_vehicle_is_turned_toward_and_reached():
    planner = make_planner((-5.0, 3.0))
    _, _, _ = drive(planner, [], steps=1200)
    assert planner.state is Bug2State.REACHED


def test_it_follows_a_wall_around_to_the_goal():
    # A wall across the m-line, 4 m wide and off to port, so the shorter way
    # around is the way Bug2 does not go.
    wall = (3.0, 4.0, -1.0, 4.0)
    planner = make_planner((8.0, 0.0))
    pose, path, closest = drive(planner, [wall])

    assert planner.state is Bug2State.REACHED
    assert planner.distance_to_goal(pose[:2]) <= planner.goal_tolerance
    # It went around the wall rather than through it.
    assert closest > 0.3
    assert max(y for _, y, _ in path) > 3.0


def test_a_goal_walled_in_on_every_side_is_reported_unreachable():
    # A closed square around the goal: no crossing of the m-line inside it
    # is ever reachable, so wall following comes back to the hit point.
    room = [
        (6.0, 10.0, -2.0, -1.5),   # south
        (6.0, 10.0, 1.5, 2.0),     # north
        (6.0, 6.5, -2.0, 2.0),     # west, the one the m-line runs into
        (9.5, 10.0, -2.0, 2.0),    # east
    ]
    planner = make_planner((8.0, 0.0))
    pose, path, closest = drive(planner, room)

    assert planner.state is Bug2State.UNREACHABLE
    assert closest > 0.3
    # It stopped back where it left the m-line, having gone all the way
    # around the outside of the room first.
    assert min(x for x, _, _ in path) < 5.0
    assert max(x for x, _, _ in path) > 10.0

    # A reachable goal assigned afterward gets driven to.
    planner.set_goal((0.0, -4.0))
    pose, _, _ = drive(planner, room, pose=pose)
    assert planner.state is Bug2State.REACHED
