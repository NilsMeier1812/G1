"""
mola_launcher.launch.py — MOLA-LiDAR-Odometrie fuer den echten G1 (Stufe 2).

Duenne Huelle um den Launch des installierten mola_lidar_odometry-Pakets
(ros2-lidar-odometry.launch.py) mit dessen Standard-Pipeline. Frueher lag
hier eine Kopie aus dem Original-g1pilot mit eigener Pipeline im ALTEN
MOLA-Format -- passt nicht zum MOLA 3.x im Image g1pilot-real-lidar.

Einstellungen fuer den G1:
  - Punkte von /livox/lidar (livox_launcher), Sensorpose = Ursprung
    (ignore_lidar_pose_from_tf): MOLA schaetzt die Bewegung des LiDAR selbst;
    mola_fixed rechnet daraus die Beckenpose (Spiegelung, Hueftdrehung).
  - Kein MOLA-GUI, kein eigenes RViz (das RViz von robot_state zeigt alles).
  - TF nur map -> mola_lidar (eigener Frame, nur zum Anschauen): base_link
    gehoert dem g1pilot-Baum (odom_unitree -> base_link kommt von mola_fixed).
  - Karte wird ab der Startpose aufgebaut: "map" = Pose des G1 beim Start.

Fehlt mola_lidar_odometry (schlankes Real-Image), startet nichts, nur ein
Hinweis -- der Rest des Real-Starts laeuft normal.
"""
import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

MOLA_ARGS = {
    'lidar_topic_name': '/livox/lidar',
    'ignore_lidar_pose_from_tf': 'True',
    'use_rviz': 'False',
    'use_mola_gui': 'False',
    'publish_localization_following_rep105': 'False',
    'mola_tf_base_link': 'mola_lidar',
    'mola_lo_reference_frame': 'map',
    'mola_state_estimator_reference_frame': 'map',
    'start_mapping_enabled': 'True',
    'start_active': 'True',
    'imu_topic_name': '/g1pilot/imu',
}


def generate_launch_description():
    try:
        share = get_package_share_directory('mola_lidar_odometry')
    except PackageNotFoundError:
        return LaunchDescription([LogInfo(msg=(
            "[MOLA] mola_lidar_odometry fehlt in diesem Image -> keine LiDAR-Pose, "
            "AUTO NAV bleibt gesperrt. Startmenue: LiDAR + Navigation (Image "
            "g1pilot-real-lidar), siehe docs/50_navigation_anleitung.md."))])
    return LaunchDescription([
        LogInfo(msg="[MOLA] LiDAR-Odometrie startet: Karte ab der Startpose (map = Start)."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'ros2-launchs', 'ros2-lidar-odometry.launch.py')),
            launch_arguments=MOLA_ARGS.items()),
    ])
