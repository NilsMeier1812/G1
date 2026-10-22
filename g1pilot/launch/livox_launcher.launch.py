"""
livox_launcher.launch.py — Livox MID-360 (G1-Kopf) -> /livox/lidar (PointCloud2).

Netzwerk-Konfiguration aus g1pilot/config/livox_mid.json, Host-IP automatisch
von ROBOT_INTERFACE (oder G1_LIDAR_HOST_IP), LiDAR-IP G1_LIDAR_IP
(siehe g1pilot/navigation/lidar_config.py). Frueher lud dieser Launch die
Werks-Datei des Treibers (192.168.1.x) -> im Roboter-LAN kamen keine Punkte an.

Fehlt livox_ros_driver2 (schlankes Real-Image ohne LiDAR), startet NICHTS und
es kommt eine Fehlermeldung -- frueher brach daran der ganze Real-Start ab
(auch Arme, Haende und loco_client).
"""
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'
################### user configure parameters for ros2 end #####################


def generate_launch_description():
    try:
        get_package_share_directory('livox_ros_driver2')
    except PackageNotFoundError:
        return LaunchDescription([LogInfo(msg=(
            "[LiDAR] livox_ros_driver2 fehlt in diesem Image -> LiDAR wird NICHT "
            "gestartet (Rest laeuft normal). Bauen mit G1_ENABLE_LIDAR=1 ./start.sh --build "
            "(Image g1pilot-real-lidar), siehe docs/50_navigation_anleitung.md."))])

    from g1pilot.navigation import lidar_config
    template = os.path.join(get_package_share_directory('g1pilot'), 'config', 'livox_mid.json')
    user_config_path, host_ip, lidar_ip, notes = lidar_config.write_config(template)

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription(
        [LogInfo(msg=f"[LiDAR] {n}") for n in notes] + [livox_driver])
