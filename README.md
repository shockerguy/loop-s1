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
index 180 is port. `vehicle_controller` subscribes and logs the nearest
reading; `apply_lidar()` is the placeholder hook where obstacle handling goes.

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
