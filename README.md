# UMD Loop 2026 || Challenge Week || Phase 1 || Problem S1

build: colcon build --packages-select waypoint_nav --symlink-install
launch: ros2 launch waypoint_nav spawn_demo.launch.py
launch alt: sh launch.sh count:=8 seed:=42 gui:=true

## Lidar

The vehicle carries a forward-facing 2D lidar on its front face: a
`gpu_lidar` sensor in `models/vehicle/model.sdf`, rendered by the
`Sensors` system in `worlds/spawn_def.sdf`. 181 beams, one per degree over
a 180 deg fan, 0.1 - 10 m.

    /vehicle/scan     sensor_msgs/LaserScan       raw, straight off the bridge
    /vehicle/depths   std_msgs/Float32MultiArray  cleaned 1D depths, in meters

`lidar_scanner` turns the first into the second, clamping the +inf / -inf /
NaN a raw scan carries into finite values so a consumer can just call `min()`.
Index 0 is the starboard (right) edge of the fan, index 90 is straight ahead,
index 180 is port. `vehicle_controller` subscribes to it and navigates off it.

Inspect it with:

    ros2 topic echo /vehicle/depths

### Rendering

Two launch defaults exist only because the GPU lidar is picky about how it
gets rendered. Both were measured on this machine (a VMware VM, SVGA II
adapter, no real GPU), parked next to a wall 1.41 m away:

  * `render_engine` (GUI) stays `ogre`, but the *server* uses `ogre2`
    via `sensor_render_engine`. Under ogre (OGRE 1.x) every beam comes back
    clamped to `range_min` no matter what is in front of it.
  * `software_gl:=true` forces Mesa's llvmpipe. Under the VMware SVGA driver
    the lidar detected nothing on 0 of 40 frames; with llvmpipe it detected
    on 40 of 40.

On a machine with a real GPU, `software_gl:=false` is faster. If the scan
ever looks healthy but reads 10 m on every beam forever, that is this
problem, not an empty world.

## Navigation

`vehicle_controller` drives to a goal point with **Bug2**, off nothing but
`/vehicle/depths` and `/vehicle/odom`. There is no map.

The *m-line* is the straight segment from where the vehicle started to the
goal. The vehicle drives along it until its forward view is obstructed, then
follows the obstacle's edge until the m-line turns up again closer to the
goal than the point it left from:

  * **go_to_goal** - turn onto the bearing of the goal and drive. A blocked
    forward sector records a *hit point* and starts wall following.
  * **turn_ccw** - rotate counter-clockwise in place until the front clears.
    That is also what puts the wall off to starboard.
  * **follow_wall** - front clear and wall to starboard: drive straight.
    Wall gone from starboard: curve clockwise (`turn_radius`) to find where
    it went. Front blocked again: back to `turn_ccw`. Wall closer than
    `min_side_distance`: drive on, but bend away from it.
  * **back_up** - not a Bug2 state. Commanded to move but going nowhere for
    `stuck_timeout`, the vehicle reverses `backup_distance` and resumes.
  * **reached** - the center of the vehicle is within `goal_tolerance`
    (default 1 m, its own width) of the goal. It stops.
  * **unreachable** - wall following came all the way back around to the hit
    point without ever crossing the m-line closer to the goal, so the
    obstacle encloses the vehicle or the goal. It stops and says why.
    Wall following also has a budget, `follow_limit` meters off the m-line,
    and spending it ends the run the same way. Bug2 only promises to
    terminate while it follows one obstacle's boundary; among scattered
    walls the vehicle hands off from one to the next, and it can settle into
    a circuit around a group of them that never passes the hit point. That
    was observed here, not imagined: 14 walls, a goal in the far corner, and
    a vehicle that orbited between 12 m and 19 m from it for five minutes.

Three details are not in the textbook version, and the vehicle does not work
without them. A CCW turn hands back to wall following only once the front is
clear by `obstacle_distance` **plus `clear_margin`**; leaving at exactly the
threshold means the first step forward trips it again, and the vehicle
chatters in place. And the body is 2 x 1 m, so its corners sweep 1.118 m when
it turns in place - more than the front-mounted lidar can see to the side. A
wall inside `min_side_distance` to starboard is therefore one the vehicle
would catch on mid-turn, so it drives on but bends away until it has room.
When it gets caught anyway - Bug2 assumes a point that can always turn on the
spot, and a box wedged in a corner cannot - the vehicle notices it has been
commanded to move for `stuck_timeout` without going anywhere, reverses
`backup_distance` blind into ground it just drove over, and carries on from
there.

The depth array is split by bearing into `right` (-90..-20 deg), `forward`
(-20..20) and `left` (20..90), each summarized by its closest beam. A beam is
obstructed below `obstacle_distance` in front and `wall_distance` to the
side. The narrow forward sector is deliberate: it keeps a wall the vehicle is
driving *past* from reading as one it is about to hit. Right-hand wall
following never consults `left`; it is computed for the logs.

By default `waypoint_generator` (below) picks the goals. With it off, pick
the goal at launch, or retarget a running vehicle over a topic:

    ros2 launch waypoint_nav spawn_demo.launch.py waypoints:=false goal_x:=8.0 goal_y:=-4.0
    ros2 topic pub --once /vehicle/goal geometry_msgs/msg/Point "{x: -6.0, y: 3.0}"

The controller publishes its state (`go_to_goal`, `reached`,
`unreachable`, ...) on `/vehicle/nav_state` (`std_msgs/String`) whenever it
changes.

Useful knobs: `goal_tolerance`, `linear_speed`, `turn_speed` (in-place yaw
rate), `turn_radius` (the clockwise hunting arc), `obstacle_distance`,
`clear_margin`, `wall_distance`, `min_side_distance`, `stuck_timeout`,
`follow_limit`. `ros2 launch waypoint_nav spawn_demo.launch.py -s` lists
them all, and the module docstring in `waypoint_nav/vehicle_controller.py`
documents the rest, including the m-line and loop-detection tolerances.

The planner itself (`Bug2Planner`) takes a pose and three sector depths and
returns a body-frame velocity, with no ROS types involved, so `test/`
exercises the whole algorithm - including full runs around a wall and around
a walled-in goal - without starting a graph:

    colcon test --packages-select waypoint_nav

## Waypoints

`waypoint_generator` keeps the vehicle busy indefinitely. It draws a
waypoint uniformly from x, y in [-12, 12] (`waypoint_bound`), rejecting any
point closer than 1.5 m (`wall_clearance`) to a wall's footprint or closer
than 2 m to the vehicle. It publishes the waypoint on `/vehicle/goal` and
marks it on the ground with a green disc 0.3 m in radius. Once the vehicle's
center is within `goal_tolerance` of the waypoint, it draws the next one by
the same rules.

    /walls              std_msgs/Float64MultiArray  N x 5 rows: x, y, yaw, length, thickness
    /vehicle/goal       geometry_msgs/Point         current waypoint, to vehicle_controller
    /vehicle/nav_state  std_msgs/String             controller state, from vehicle_controller

  * **Walls** come from `random_cube_spawner`, which publishes every wall it
    has requested on `/walls`. The topic is transient local, so a late
    subscriber still gets the full list. A wall added later through
    `~/spawn_cube` republishes the list, and a waypoint the new wall crowds
    is replaced.
  * **Giving up** - when the controller reports `unreachable`, the generator
    draws a new waypoint instead of leaving the vehicle parked.
  * **Hand-off** - the controller's goal subscription is volatile, so the
    generator re-sends the current waypoint whenever a `vehicle_controller`
    subscription appears. It ignores other subscribers, such as
    `ros2 topic echo`, because every goal restarts Bug2.
  * **Marker** - a static, visual-only model: the vehicle drives over it, and
    it sits well below the lidar's scan plane. It is spawned once through
    `/world/spawn_demo/create` and moved through `/world/spawn_demo/set_pose`.

Launch arguments: `waypoints` (default `true`), `waypoint_bound`,
`wall_clearance`, `waypoint_seed` (-1 for random; the seed is logged).

    ros2 launch waypoint_nav spawn_demo.launch.py seed:=42 waypoint_seed:=5
