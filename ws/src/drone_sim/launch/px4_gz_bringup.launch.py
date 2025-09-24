# px4_gz_bringup.launch.py — Gazebo (gz) Harmonic + PX4 SITL bringup

import os
from shutil import which

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
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
    world_name = LaunchConfiguration('world_name')
    px4_verbose = LaunchConfiguration('verbose')

    world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_path, 'worlds', 'empty.world'),
        description='Path to world SDF'
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='SDF <world name="..."> (e.g. empty, default)'
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
        'px4_sim_model', default_value='',
        description='PX4 will spawn this model folder (e.g., x500). Leave empty to attach to existing model.'
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
        'verbose', default_value='4',
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
    render_engine = SetEnvironmentVariable(name='GZ_RENDER_ENGINE', value='ogre2')

    # -------------------------
    # Headless/GUI env
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
    # Prime Gazebo server.config (since --server-config is unsupported)
    # -------------------------
    server_config = os.path.join(px4_gz_root, 'server.config')
    prime_server_cfg = ExecuteProcess(
        cmd=['bash', '-lc',
             f'mkdir -p ~/.gz/sim/8 && cp -f "{server_config}" ~/.gz/sim/8/server.config && '
             'echo "[gz] primed ~/.gz/sim/8/server.config"'],
        output='screen'
    )

    # -------------------------
    # Start Gazebo (gz sim)
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

    # Start Gazebo only after the prime step finishes
    start_gz_after_prime = RegisterEventHandler(
        OnProcessExit(target_action=prime_server_cfg, on_exit=[gazebo_headless, gazebo_gui])
    )

    # -------------------------
    # Explicitly unpause the world (safe if already unpaused)
    # -------------------------
    unpause_world = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'for i in $(seq 1 80); do '
            '  gz service -s /world/${WORLD_NAME}/control '
            '    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean '
            "    --req 'pause: false' >/dev/null 2>&1 && break; "
            '  sleep 0.1; '
            'done; '
            'echo "[unpause] Requested pause:false on /world/${WORLD_NAME}/control"'
        ],
        output='screen',
        additional_env={'WORLD_NAME': world_name},
    )

    # -------------------------
    # Wait for first IMU and Baro messages (prevents STALE)
    # -------------------------
    wait_imu_then_baro = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'STATS="/world/${WORLD_NAME}/stats"; '
            'echo "[wait] Waiting for world stats on ${STATS}…"; '
            '(gz topic -e -t "$STATS" -n 1 >/dev/null 2>&1) || true; '

            # Choose the single model name to watch: attach name or spawned name
            'MODEL="${PX4_GZ_MODEL_NAME}"; '
            'if [ -z "$MODEL" ]; then MODEL="${PX4_SIM_MODEL}_0"; fi; '
            'IMU="/world/${WORLD_NAME}/model/${MODEL}/link/base_link/sensor/imu_sensor/imu"; '
            'BARO="/world/${WORLD_NAME}/model/${MODEL}/link/base_link/sensor/air_pressure_sensor/air_pressure"; '
            'echo "[wait] IMU  topic: $IMU"; '
            'echo "[wait] BARO topic: $BARO"; '

            # Wait until topics exist
            'until gz topic -i -t "$IMU"  >/dev/null 2>&1; do sleep 0.1; done; '
            'until gz topic -i -t "$BARO" >/dev/null 2>&1; do sleep 0.1; done; '
            'echo "[wait] Topics exist; waiting for first messages…"; '

            # Wait one actual message from each
            '(gz topic -e -t "$IMU"  -n 1 >/dev/null 2>&1); '
            '(gz topic -e -t "$BARO" -n 1 >/dev/null 2>&1); '
            'echo "[wait] First IMU + BARO messages received."'
        ],
        output='screen',
        additional_env={
            'WORLD_NAME': world_name,
            'PX4_GZ_MODEL_NAME': px4_gz_model_name,
            'PX4_SIM_MODEL': px4_sim_model,
        },
    )

    # -------------------------
    # PX4 SITL (starts only after waits)
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
            # Choose ONE per run: either PX4_SIM_MODEL (spawn) OR PX4_GZ_MODEL_NAME (attach)
            'PX4_SIM_MODEL': px4_sim_model,
            'PX4_GZ_MODEL_NAME': px4_gz_model_name,
        },
        condition=IfCondition(px4)
    )

    # Chain: prime -> start gz -> unpause -> wait sensors -> PX4
    start_wait_after_unpause = RegisterEventHandler(
        OnProcessExit(target_action=unpause_world, on_exit=[wait_imu_then_baro])
    )
    start_px4_after_waits = RegisterEventHandler(
        OnProcessExit(target_action=wait_imu_then_baro, on_exit=[px4_proc])
    )

    # -------------------------
    # Bridges (Gazebo -> ROS 2) — optional / for visualization
    # NOTE: PX4 does NOT read these; they are for ROS only.
    # -------------------------
    bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/x500_custom/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/x500_custom/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
            '/x500_custom/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/x500_custom/lidar/front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/x500_custom/lidar/side@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    img_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/x500_custom/camera/image'],
        output='screen',
    )

    pin_gz_ip = SetEnvironmentVariable(name='GZ_IP', value='127.0.0.1')
    pin_gz_partition  = SetEnvironmentVariable(name='GZ_PARTITION', value='px4')
    pin_gz_disc_port  = SetEnvironmentVariable(name='GZ_DISCOVERY_PORT', value='11346')
    pin_gz_pub_port   = SetEnvironmentVariable(name='GZ_PUBLISH_PORT',   value='11347')
    pin_gz_sub_port   = SetEnvironmentVariable(name='GZ_SUBSCRIBE_PORT', value='11348')

    return LaunchDescription([
        # Args
        world_arg, world_name_arg, headless_arg, px4_arg,
        px4_sim_model_arg, px4_gz_model_name_arg, px4_sys_autostart_arg, verbose_arg,
        # Env
        set_gz_path, render_engine,
        soft_gl, disable_gui, qt_offscreen,
        enable_gui, qt_xcb,
        pin_gz_ip, pin_gz_partition, pin_gz_disc_port, pin_gz_pub_port, pin_gz_sub_port,
        # Prime -> Gazebo
        prime_server_cfg,
        start_gz_after_prime,
        # Unpause -> wait -> PX4
        unpause_world,
        start_wait_after_unpause,
        start_px4_after_waits,
        # Optional ROS bridges (not needed for PX4/EKF)
        # bridges, img_bridge,
    ])
