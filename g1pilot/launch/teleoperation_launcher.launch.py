import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Welche Bedienoberflaeche starten? 'streamdeck' (ui_interface, Default) oder
# 'demo' (demo_gui, vereinfacht fuer Vorfuehrungen). Nie beide: jede GUI macht
# in der Sim ihren eigenen Auto-Start. Siehe g1pilot/docs/42_demo_gui_konzept.md.
GUI_EXECUTABLES = {'streamdeck': 'ui_interface', 'demo': 'demo_gui'}


def generate_launch_description():
    joystick_name = LaunchConfiguration('joystick_name')
    ps4_arms = LaunchConfiguration('ps4_arms')
    gui = os.environ.get('G1_GUI', 'streamdeck').strip().lower()
    gui_exe = GUI_EXECUTABLES.get(gui, 'ui_interface')
    return LaunchDescription([
        DeclareLaunchArgument(
            'joystick_name', default_value='Pro Controller',
            description='Name of the joystick device to bind to'),
        # PS4-Controller steuert den OBERKOERPER (Arme + Haende), nicht das
        # Laufen. Nur bringup_sim setzt das (G1_PS4_ARMS=1); real bleibt false.
        # Dann laeuft der alte joystick-Node NICHT: er laese denselben
        # Controller und schickte die Sticks ueber joy_mux an das Laufen.
        # Siehe g1pilot/docs/43_ps4_controller.md.
        DeclareLaunchArgument(
            'ps4_arms', default_value='false',
            description='PS4-Controller fuer Arme/Haende statt fuer joy_mux'),

        Node(
            package='g1pilot',
            executable='joystick',
            name='joystick',
            output='screen',
            parameters=[
                {'joystick_name': joystick_name}],
            condition=UnlessCondition(ps4_arms),
        ),

        Node(
            package='g1pilot',
            executable='ps4_joystick',
            name='ps4_joystick',
            output='screen',
            parameters=[
                {'joystick_name': joystick_name}],
            condition=IfCondition(ps4_arms),
        ),

        Node(
            package='g1pilot',
            executable='ps4_arm_teleop',
            name='ps4_arm_teleop',
            output='screen',
            condition=IfCondition(ps4_arms),
        ),

        Node(
            package='g1pilot',
            executable='joy_mux',
            name='joy_mux',
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable=gui_exe,
            name=gui_exe,
            output='screen'
        ),

    ])
