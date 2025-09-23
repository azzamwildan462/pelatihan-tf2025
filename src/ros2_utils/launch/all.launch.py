import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

from ament_index_python.packages import get_package_share_directory

path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"

def generate_launch_description():

    # Default menggunakan cyclone dds (localhost)
    SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
    SetEnvironmentVariable(name='CYCLONEDDS_URI', value='file://' + path_config + 'cyclonedds.xml'),

    # ============================== TFs ===================================
    tf_base_link_to_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_camera_link",
        # fmt: off
        arguments=["-0.14","0.08","1.35","0.00","0.045","0.00","base_link","camera_link",
                   "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_lidar_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_lidar_link",
        # fmt: off
        arguments=["0.14","0.00","0.35","1.57","0.00","0.00","base_link","lidar_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    # ======================================================================

    # ========================== Bawaan ROS ================================
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        respawn=True,
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi_node',
        output='screen',
        respawn=True,
    )

    realsense2_camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera_node",
        parameters=[
            {
                "enable_color": True,
                "enable_depth": True,
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_rgbd": False,
                "enable_mag": False,
                "enable_sync": True,
                
                "enable_accel": True,
                "enable_gyro": True,
                "unite_imu_method": 2,
                "align_depth.enable": True,
                "pointcloud.enable": True,
                "initial_reset": False,
                "gyro_fps": 200,
                "accel_fps": 200,
            }
        ],
        remappings=[("/imu", "/imu_raw")],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
    )

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        parameters=[{"use_mag": False}],
        remappings=[
            ("/imu/data_raw", "/camera/realsense2_camera_node/imu"),
            ("/imu/data", "/imu_filtered"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
    )

    rtabmap_slam_rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="slam",
        parameters=[
            {
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",

                "use_sim_time": False,
                "Threads": 9, # Added by Azzam
            }
        ],
        remappings=[
            ("rgb/image", "/camera/realsense2_camera_node/color/image_raw"),
            ("rgb/camera_info", "/camera/realsense2_camera_node/color/camera_info"),
            ("depth/image", "/camera/realsense2_camera_node/aligned_depth_to_color/image_raw"),
            ("scan", "/laserscan"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    ds4_driver_node = Node(
        package="ds4_driver",
        executable="ds4_driver_node.py",
        name="ds4_driver_node",
        remappings=[
            ("/status", "/ds4/to_pc"),
            ("/set_feedback", "/ds4/from_pc"),
        ],
        respawn=True,
    )

    # ======================================================================

    # ========================== Communication ================================
    wifi_control = Node(
        package="communication",
        executable="wifi_control",
        name="wifi_control",
        parameters=[
            {
                "hotspot_ssid": "gh_template",
                "hotspot_password": "gh_template",
            },
        ],
        output="screen",
        respawn=True,
    )

    telemetry = Node(
        package="communication",
        executable="telemetry.py",
        name="telemetry",
        parameters=[{
            "INFLUXDB_URL": "http://172.30.37.21:8086",
            "INFLUXDB_USERNAME": "awm462",
            "INFLUXDB_PASSWORD": "asdasdasd",
            "INFLUXDB_ORG": "awmawm",
            "INFLUXDB_BUCKET": "ujiCoba",
            "ROBOT_NAME": "gh_template",
        }],
        output="screen",
        respawn=True,
    )

    # ============================ Hardware ================================
    keyboard_input = Node(
        package='hardware',
        executable='keyboard_input',
        name='keyboard_input',
        output='screen',
        respawn=True,
        prefix=['xterm -e'],
    )

    io_stm32 = Node(
        package="hardware",
        executable="io_stm32",
        name="io_stm32",
        parameters=[
            {
                "ip": "192.168.50.2",
                "port": 9798,
            }
        ],
        output="screen",
        respawn=True,
    )

    io_lslidar_n301 = Node(
        package="hardware",
        executable="io_lslidar_n301",
        name="io_lslidar_n301",
        parameters=[
            {
                "port_msop": 2368,
                "port_difop": 2369,
                "frame_id": "lidar_link",
                "distance_min": 0.2,
                "distance_max": 20.0,
                "azimuth_start": 185.0,
                "azimuth_stop": 355.0,
                "azimuth_step": 0.05,
            }
        ],
        output="screen",
        respawn=True,
    )

    # ============================ Master ================================
    master = Node(
        package='master',
        executable='master',
        name='master',
        output='screen',
        respawn=True,
        prefix='nice -n -10',
    )

    # ============================ WEB UI ================================
    ui_server = Node(
        package="web_ui",
        executable="ui_server.py",
        name="ui_server",
        parameters=[
            {
                "ui_root_path": os.path.join(ws_path,"src/web_ui/src")
            },
        ],
        output="screen",
        respawn=True,
    )

    return LaunchDescription(
        [
            # rosapi_node,
            # ui_server,
            # rosbridge_server, 

            # telemetry,

            master,

            # keyboard_input,

            # wifi_control,
        ]
    )
