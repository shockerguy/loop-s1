"""Bring up Gazebo Harmonic, the bridge, the wall spawner, the vehicle, and lidar.

The vehicle's front lidar is a gpu_lidar sensor in models/vehicle/model.sdf,
rendered by the world's Sensors system and bridged out as
/vehicle/scan (sensor_msgs/LaserScan). lidar_scanner cleans that scan up into
/vehicle/depths (std_msgs/Float32MultiArray), which vehicle_controller reads
alongside /vehicle/odom to drive itself to a goal with Bug2.

waypoint_generator supplies that goal: a random point in
[-waypoint_bound, waypoint_bound]^2 at least wall_clearance from every wall,
marked on the ground by a green disc and replaced with a new one each time
the vehicle gets within goal_tolerance of it. waypoints:=false turns it off,
and the vehicle drives to goal_x, goal_y once instead.

    ros2 launch waypoint_nav spawn_demo.launch.py
    ros2 launch waypoint_nav spawn_demo.launch.py count:=8 seed:=42
    ros2 launch waypoint_nav spawn_demo.launch.py waypoint_seed:=7
    ros2 launch waypoint_nav spawn_demo.launch.py waypoints:=false goal_x:=8.0 goal_y:=-4.0

Retarget a running vehicle without relaunching (with waypoints:=false, or
the generator's next waypoint overrides it):

    ros2 topic pub --once /vehicle/goal geometry_msgs/msg/Point "{x: -6.0, y: 3.0}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Must match <world name="..."> inside worlds/spawn_def.sdf. The Gazebo
# service path is derived from it, so a mismatch shows up as the spawner
# waiting forever on a service that never appears.
WORLD_NAME = 'spawn_demo'

# Must match <min_angle>/<max_angle> on the front_lidar sensor in
# models/vehicle/model.sdf. Not a launch argument, because changing it here
# alone would only change how vehicle_controller labels a beam's bearing, not
# where the sensor actually looks.
LIDAR_FOV_DEG = 180.0


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('waypoint_nav')
    world_file = os.path.join(pkg_share, 'worlds', 'spawn_def.sdf')

    # Lets the world's <include><uri>model://vehicle</uri> resolve.
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models'))

    # The GPU lidar renders its beams through OpenGL. Under VMware's SVGA
    # driver that render silently yields nothing, so every beam reads as "no
    # return" and the vehicle is blind while the scan still looks healthy.
    # Mesa's llvmpipe software rasterizer returns correct depths on every
    # frame. Slower, but a 181-beam 2D scan is cheap. Turn this off on a
    # machine with a real GPU.
    software_gl = [
        SetEnvironmentVariable(
            name, value,
            condition=IfCondition(LaunchConfiguration('software_gl')))
        for name, value in (('LIBGL_ALWAYS_SOFTWARE', '1'),
                            ('GALLIUM_DRIVER', 'llvmpipe'))
    ]

    gui = LaunchConfiguration('gui').perform(context)
    render_engine = LaunchConfiguration('render_engine').perform(context)
    sensor_engine = LaunchConfiguration('sensor_render_engine').perform(context)
    # Server and GUI are set separately on purpose. The GUI stays on ogre
    # (OGRE 1.x) because Gazebo's ogre2 default flickers on some setups
    # (notably VMs), but ogre1's GPU lidar is broken here: every beam comes
    # back clamped to range_min regardless of what is in front of it. The
    # server is what renders sensors, so it gets ogre2 and the lidar works.
    # Plain --render-engine would set both at once and reintroduce that.
    gz_args = (f'-r -v 3 --render-engine-gui {render_engine} '
               f'--render-engine-server {sensor_engine} {world_file}')
    if gui.lower() in ('false', '0'):
        gz_args = f'-s {gz_args}'  # server only, useful for CI

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_service_bridge',
        arguments=[
            f'/world/{WORLD_NAME}/create@ros_gz_interfaces/srv/SpawnEntity',
            # Moves the waypoint marker.
            f'/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Topic names are set in models/vehicle/model.sdf.
            '/vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/vehicle/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/vehicle/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    spawner = Node(
        package='waypoint_nav',
        executable='random_cube_spawner',
        name='random_cube_spawner',
        parameters=[{
            'world_name': WORLD_NAME,
            'count': LaunchConfiguration('count'),
            'seed': LaunchConfiguration('seed'),
            'bound': LaunchConfiguration('bound'),
            'size_x_min': LaunchConfiguration('size_x_min'),
            'size_x_max': LaunchConfiguration('size_x_max'),
            'size_y': LaunchConfiguration('size_y'),
            'size_z': LaunchConfiguration('size_z'),
            'static': LaunchConfiguration('static'),
            'clear_radius': LaunchConfiguration('clear_radius'),
        }],
        output='screen',
    )

    lidar = Node(
        package='waypoint_nav',
        executable='lidar_scanner',
        name='lidar_scanner',
        output='screen',
    )

    waypoints = Node(
        package='waypoint_nav',
        executable='waypoint_generator',
        name='waypoint_generator',
        parameters=[{
            'world_name': WORLD_NAME,
            'bound': LaunchConfiguration('waypoint_bound'),
            'wall_clearance': LaunchConfiguration('wall_clearance'),
            # Same radius the controller stops at, so a new waypoint appears
            # exactly when the vehicle arrives at the old one.
            'reach_radius': LaunchConfiguration('goal_tolerance'),
            'seed': LaunchConfiguration('waypoint_seed'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('waypoints')),
    )

    controller = Node(
        package='waypoint_nav',
        executable='vehicle_controller',
        name='vehicle_controller',
        parameters=[{
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'turn_radius': LaunchConfiguration('turn_radius'),
            'obstacle_distance': LaunchConfiguration('obstacle_distance'),
            'clear_margin': LaunchConfiguration('clear_margin'),
            'wall_distance': LaunchConfiguration('wall_distance'),
            'min_side_distance': LaunchConfiguration('min_side_distance'),
            'follow_limit': LaunchConfiguration('follow_limit'),
            'lidar_fov_deg': LIDAR_FOV_DEG,
        }],
        output='screen',
    )

    # Give the Gazebo server a moment to advertise its services before the
    # bridge goes looking for them. The spawner polls, so it can start with
    # the bridge.
    delayed = TimerAction(period=4.5,
                          actions=[bridge, spawner, lidar, waypoints])

    # Hold the vehicle still until the walls are in. clear_radius only keeps
    # them off the origin, so a wall arriving after the vehicle has moved
    # could still land on top of it.
    drive = TimerAction(period=7.0, actions=[controller])

    return [resource_path, *software_gl, gazebo, delayed, drive]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='8',
                              description='Cubes to spawn at startup'),
        DeclareLaunchArgument('seed', default_value='-1',
                              description='RNG seed; -1 for a random one'),
        DeclareLaunchArgument('bound', default_value='10.0',
                              description='x and y range is [-bound, bound]'),
        DeclareLaunchArgument('size_x_min', default_value='1.0',
                              description='Minimum wall length (x) in meters'),
        DeclareLaunchArgument('size_x_max', default_value='4.0',
                              description='Maximum wall length (x) in meters'),
        DeclareLaunchArgument('size_y', default_value='1.0',
                              description='Wall thickness (y) in meters'),
        DeclareLaunchArgument('size_z', default_value='2.0',
                              description='Wall height (z) in meters'),
        DeclareLaunchArgument('static', default_value='true',
                              description='Immovable walls if true'),
        DeclareLaunchArgument('clear_radius', default_value='2.0',
                              description='Keep walls this far from the '
                                          'vehicle spawn at the origin (m)'),
        DeclareLaunchArgument('waypoints', default_value='true',
                              description='Drive to an endless series of '
                                          'random waypoints; false drives '
                                          'to goal_x, goal_y once'),
        DeclareLaunchArgument('waypoint_bound', default_value='12.0',
                              description='Waypoint x and y range is '
                                          '[-waypoint_bound, waypoint_bound]'),
        DeclareLaunchArgument('wall_clearance', default_value='1.5',
                              description='Keep waypoints at least this far '
                                          'from any wall (m)'),
        DeclareLaunchArgument('waypoint_seed', default_value='-1',
                              description='Waypoint RNG seed; -1 for a '
                                          'random one'),
        DeclareLaunchArgument('goal_x', default_value='8.0',
                              description='Goal x in the odom frame (m); '
                                          'only used until the first '
                                          'waypoint arrives'),
        DeclareLaunchArgument('goal_y', default_value='0.0',
                              description='Goal y in the odom frame (m); '
                                          'see goal_x'),
        DeclareLaunchArgument('goal_tolerance', default_value='1.0',
                              description='Goal reached inside this radius '
                                          '(m); default is the width of the '
                                          'vehicle'),
        DeclareLaunchArgument('linear_speed', default_value='1.0',
                              description='Vehicle forward speed (m/s)'),
        DeclareLaunchArgument('turn_speed', default_value='0.6',
                              description='Yaw rate when turning in place '
                                          '(rad/s)'),
        DeclareLaunchArgument('turn_radius', default_value='2.0',
                              description='Radius (m) of the clockwise arc '
                                          'the vehicle takes to hunt for a '
                                          'wall that has ended'),
        DeclareLaunchArgument('obstacle_distance', default_value='1.5',
                              description='Forward sector counts as blocked '
                                          'below this distance (m)'),
        DeclareLaunchArgument('clear_margin', default_value='0.5',
                              description='Extra clearance (m) the front '
                                          'needs before a CCW turn hands '
                                          'back to wall following; '
                                          'hysteresis, 0 disables'),
        DeclareLaunchArgument('wall_distance', default_value='1.5',
                              description='Right sector counts as blocked '
                                          'below this distance (m), which is '
                                          'how far off a wall it follows'),
        DeclareLaunchArgument('min_side_distance', default_value='0.8',
                              description='Bend away from a wall closer than '
                                          'this (m) to starboard, which the '
                                          'corners would otherwise catch on '
                                          'when turning in place'),
        DeclareLaunchArgument('follow_limit', default_value='60.0',
                              description='Give up after wall following this '
                                          'far (m) without rejoining the '
                                          'line to the goal; 0 for no limit'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run Gazebo with its GUI'),
        DeclareLaunchArgument('render_engine', default_value='ogre',
                              description='GUI render engine: ogre or ogre2'),
        DeclareLaunchArgument('software_gl', default_value='true',
                              description='Render through Mesa llvmpipe. The '
                                          'VMware SVGA driver returns nothing '
                                          'for the lidar; set false if this '
                                          'machine has a real GPU'),
        DeclareLaunchArgument('sensor_render_engine', default_value='ogre2',
                              description='Server render engine, which is what '
                                          'draws the lidar. ogre returns '
                                          'range_min on every beam, so this '
                                          'wants ogre2'),
        OpaqueFunction(function=launch_setup),
    ])
