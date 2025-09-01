# px4_gz_bringup.launch.py — Gazebo (gz) Garden/Harmonic + PX4 SITL bringup
# Launches Gazebo via 'gz sim' (not ign), supports headless & GUI, then starts PX4 + bridges.

import os
from shutil import which

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -------------------------
    # Paths
    # -------------------------
    pkg_path = get_package_share_directory('drone_sim')
    px4_root = os.environ.get('PX4_ROOT', '/repo/ws/src/px4')
    px4_gz_root = os.path.join(px4_root, 'Tools', 'simulation', 'gz')

    # -------------------------
    # Launch arguments
    # -------------------------
    headless = LaunchConfiguration('headless')
    px4 = LaunchConfiguration('px4')
    px4_sim_model = LaunchConfiguration('px4_sim_model')
    px4_gz_model_name = LaunchConfiguration('px4_gz_model_name')
    px4_sys_autostart = LaunchConfiguration('px4_sys_autostart')
    world_file = LaunchConfiguration('world_file')
    px4_verbose = LaunchConfiguration('verbose')


    world_arg = DeclareLaunchArgument(
    'world_file',
    default_value=os.path.join(pkg_path, 'worlds', 'empty.world'),

    description='Path to world SDF'
)
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run without GUI ("true") or with GUI ("false")'
    )
    px4_arg = DeclareLaunchArgument(
        'px4', default_value='true',
        description='Start PX4 SITL alongside Gazebo ("true" or "false")'
    )
    px4_sim_model_arg = DeclareLaunchArgument(
        'px4_sim_model', default_value='x500',
        description='PX4 will spawn this model folder (e.g., x500, x500_custom).'
    )
    px4_gz_model_name_arg = DeclareLaunchArgument(
        'px4_gz_model_name', default_value='',
        description='Bind PX4 to an existing Gazebo model name (overrides spawn if set).'
    )
    px4_sys_autostart_arg = DeclareLaunchArgument(
        'px4_sys_autostart', default_value='4001',
        description='PX4 airframe autostart ID (e.g., 4001 for X500).'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='2',
        description="PX4 verbosity - '1'=error, '2'=warn, '3'=info, '4'=debug'"
    )

    # -------------------------
    # Gazebo resources
    # -------------------------
    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=":".join([
            os.path.join(pkg_path, 'models'),
            os.path.join(px4_gz_root, 'models'),
            px4_gz_root
        ])
    )
    # Always use Ogre2
    render_engine = SetEnvironmentVariable(name='GZ_RENDER_ENGINE', value='ogre2')

    # -------------------------
    # Headless/GUI env (apply offscreen vars ONLY when headless)
    # -------------------------
    soft_gl = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE', value='1', condition=IfCondition(headless)
    )
    disable_gui = SetEnvironmentVariable(
        name='GZ_GUI', value='0', condition=IfCondition(headless)
    )
    qt_offscreen = SetEnvironmentVariable(
        name='QT_QPA_PLATFORM', value='offscreen', condition=IfCondition(headless)
    )

    enable_gui = SetEnvironmentVariable(
        name='GZ_GUI', value='1', condition=UnlessCondition(headless)
    )
    qt_xcb = SetEnvironmentVariable(
        name='QT_QPA_PLATFORM', value='xcb', condition=UnlessCondition(headless)
    )

    # -------------------------
    # Start Gazebo (force 'gz sim'; fall back to 'ign' only if gz missing)
    # -------------------------
    _sim_cmd = ['gz', 'sim'] if which('gz') else ['ign', 'gazebo']

    VERBOSE = px4_verbose

    gazebo_headless = ExecuteProcess(
        cmd=_sim_cmd + ['-r', '-s', '--headless-rendering', '-v', VERBOSE, world_file],
        output='screen',
        condition=IfCondition(headless),
    )

    gazebo_gui = ExecuteProcess(
        cmd=_sim_cmd + ['-r', '-v', VERBOSE, world_file],
        output='screen',
        condition=UnlessCondition(headless),
)


    # -------------------------
    # PX4 SITL (gz/Garden bridge)
    # -------------------------
    px4_build = os.path.join(px4_root, 'build', 'px4_sitl_default')
    px4_bin = os.path.join(px4_build, 'bin', 'px4')
    rcs_rel = 'etc/init.d-posix/rcS'  # valid when cwd=px4_build

    px4_proc = ExecuteProcess(
        cmd=[px4_bin, '-d', '-s', rcs_rel],
        cwd=px4_build,
        output='screen',
        additional_env={
            'PX4_SYS_AUTOSTART': px4_sys_autostart,
            'PX4_SIM_MODEL': px4_sim_model,          # PX4 spawns this model folder
            'PX4_GZ_MODEL_NAME': px4_gz_model_name,  # Or bind to an existing model
        },
        condition=IfCondition(px4)
    )

    # Give Gazebo time to start so /world/*/create exists
    delayed_px4 = TimerAction(period=6.0, actions=[px4_proc])

    # -------------------------
    # Bridges (Gazebo -> ROS 2)
    # -------------------------
    bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/x500_custom_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/empty/model/x500_custom_0/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
            '/world/empty/model/x500_custom_0/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/x500_custom_0/lidar/front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/x500_custom_0/lidar/side@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    img_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/x500_custom_0/camera/image'],  # include leading slash
        output='screen',
    )


    delayed_bridges = TimerAction(period=5.0, actions=[bridges, img_bridge])

    return LaunchDescription([
        # Args
        world_arg, headless_arg, px4_arg, px4_sim_model_arg, px4_gz_model_name_arg, px4_sys_autostart_arg, verbose_arg,
        # Env
        set_gz_path, render_engine,
        soft_gl, disable_gui, qt_offscreen,
        enable_gui, qt_xcb,
        # Gazebo
        gazebo_headless, gazebo_gui,
        # PX4 + bridges
        delayed_px4, delayed_bridges,
    ])
