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
