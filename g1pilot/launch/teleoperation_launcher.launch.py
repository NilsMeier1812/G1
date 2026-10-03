import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Welche Bedienoberflaeche starten? 'streamdeck' (ui_interface, Default) oder
# 'demo' (demo_gui, vereinfacht fuer Vorfuehrungen). Nie beide: jede GUI macht
# in der Sim ihren eigenen Auto-Start. Siehe g1pilot/docs/42_demo_gui_konzept.md.
GUI_EXECUTABLES = {'streamdeck': 'ui_interface', 'demo': 'demo_gui'}


def generate_launch_description():
    joystick_name = LaunchConfiguration('joystick_name')
    gui = os.environ.get('G1_GUI', 'streamdeck').strip().lower()
    gui_exe = GUI_EXECUTABLES.get(gui, 'ui_interface')
    return LaunchDescription([
        DeclareLaunchArgument(
            'joystick_name', default_value='Pro Controller',
            description='Name of the joystick device to bind to'),

        Node(
            package='g1pilot',
            executable='joystick',
            name='joystick',
            output='screen',
            parameters=[
                {'joystick_name': joystick_name}],
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
