import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Bedienoberflaeche ist die Demo-GUI (demo_gui), siehe g1pilot/docs/10-A-Demo-GUI.md.
# Die alte Streamdeck-GUI (ui_interface) wurde entfernt.


def generate_launch_description():
    joystick_name = LaunchConfiguration('joystick_name')
    ps4_arms = LaunchConfiguration('ps4_arms')
    ps4_params = {
        'speed': LaunchConfiguration('ps4_speed'),
        'max_speed': LaunchConfiguration('ps4_max_speed'),
        'options_starts_balancing': ParameterValue(
            LaunchConfiguration('ps4_options_balancing'), value_type=bool),
        'ps_estop': ParameterValue(LaunchConfiguration('ps4_ps_estop'), value_type=bool),
        # L2 = Gehen (ps4_walk): Arm-Steuerung pausiert, solange L2 gehalten.
        'walk_deadman': ParameterValue(LaunchConfiguration('ps4_walk'), value_type=bool),
        # L2 druecken = Modus GEHEN (danach halten = laufen), R2 = GREIFEN.
        'trigger_modes': ParameterValue(LaunchConfiguration('ps4_walk'), value_type=bool),
    }
    ps4_walk_on = PythonExpression(["'", ps4_arms, "'.lower() == 'true' and '",
                                    LaunchConfiguration('ps4_walk'), "'.lower() == 'true'"])
    return LaunchDescription([
        DeclareLaunchArgument(
            'joystick_name', default_value='Pro Controller',
            description='Name of the joystick device to bind to'),
        # PS4-Controller steuert den OBERKOERPER (Arme + Haende), Gehen nur
        # ueber ps4_walk (L2-Deadman, eigenes Topic /g1pilot/ps4/walk_cmd).
        # bringup_sim und bringup_real setzen das aus G1_PS4_ARMS=1.
        # Dann laeuft der alte joystick-Node NICHT: er laese denselben
        # Controller und schickte die Sticks ueber joy_mux an das Laufen
        # (Sim: joy_to_cmdvel, Real: loco_client mit Deadman).
        # Siehe g1pilot/docs/11-A-Controller.md.
        DeclareLaunchArgument(
            'ps4_arms', default_value='false',
            description='PS4-Controller fuer Arme/Haende statt fuer joy_mux'),
        # Defaults = Sim; bringup_real setzt vorsichtigere Werte.
        DeclareLaunchArgument('ps4_speed', default_value='Normal',
                              description='Start-Tempo: Langsam | Normal | Schnell'),
        DeclareLaunchArgument('ps4_max_speed', default_value='Schnell',
                              description='Hoechstes Tempo per R1'),
        DeclareLaunchArgument('ps4_options_balancing', default_value='true',
                              description='Options sendet zusaetzlich start_balancing'),
        DeclareLaunchArgument('ps4_ps_estop', default_value='true',
                              description='PS = NOT-HALT (false: nur Stopp wie Kreuz)'),
        # Gehen per PS4 (nur mit ps4_arms): L2 halten = Deadman, Sticks gehen.
        DeclareLaunchArgument('ps4_walk', default_value='true',
                              description='L2 + Sticks = Gehen (ps4_walk)'),
        DeclareLaunchArgument('ps4_walk_speed', default_value='Langsam',
                              description='Start-Geh-Tempo: Langsam | Normal | Schnell'),
        DeclareLaunchArgument('ps4_walk_max_speed', default_value='Schnell',
                              description='Hoechstes Geh-Tempo per L2 + Steuerkreuz oben'),

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
                {'joystick_name': joystick_name,
                 # PS4 oder Xbox (gleiche Belegung nach Position), siehe
                 # g1pilot/docs/11-A-Controller.md#xbox-controller
                 'controller': os.environ.get('G1_CONTROLLER', 'auto').strip().lower() or 'auto',
                 'xbox_ps_button': os.environ.get('G1_XBOX_PS_BUTTON', 'xbox').strip().lower() or 'xbox'}],
            condition=IfCondition(ps4_arms),
        ),

        Node(
            package='g1pilot',
            executable='ps4_arm_teleop',
            name='ps4_arm_teleop',
            output='screen',
            parameters=[ps4_params],
            condition=IfCondition(ps4_arms),
        ),

        Node(
            package='g1pilot',
            executable='ps4_walk',
            name='ps4_walk',
            output='screen',
            parameters=[{
                'speed': LaunchConfiguration('ps4_walk_speed'),
                'max_speed': LaunchConfiguration('ps4_walk_max_speed'),
                'ps_estop': ParameterValue(LaunchConfiguration('ps4_ps_estop'),
                                           value_type=bool),
            }],
            condition=IfCondition(ps4_walk_on),
        ),

        Node(
            package='g1pilot',
            executable='joy_mux',
            name='joy_mux',
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='demo_gui',
            name='demo_gui',
            output='screen'
        ),

    ])
