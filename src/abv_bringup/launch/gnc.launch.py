from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix

def generate_launch_description():

    abv_rl_script = PathJoinSubstitution([
        FindPackagePrefix('abv_rl'), 'lib', 'abv_rl', 'abv_rl_venv.sh'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('rl', default_value='false'),
        # static roll (deg) the passive half is mounted at, copied into every
        # docking log row (SYS.09 runs), e.g. fixture_roll_deg:=4.0
        DeclareLaunchArgument('fixture_roll_deg', default_value='0.0'),

        Node(
            package='abv_goal_manager',
            executable='abv_goal_manager',
            name='abv_goal_manager',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_control',
            executable='abv_control',
            name='abv_control',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_thrusters',
            executable='abv_thrusters',
            name='abv_thrusters',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_navigation',
            executable='abv_navigation',
            name='abv_navigation',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_guidance',
            executable='abv_guidance',
            name='abv_guidance',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_bridge',
            executable='abv_bridge',
            name='abv_ptera_bridge',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_policy_registry',
            executable='abv_policy_registry',
            name='abv_policy_registry',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_rl-cpp',
            executable='abv_rl-cpp',
            name='rl_policy_node',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('rl')),
        ),

        # Per-cycle docking log (TEST.09), sim and hardware alike
        Node(
            package='abv_dock_bridge',
            executable='dock_logger',
            name='abv_dock_logger',
            output='screen',
            emulate_tty=True,
            parameters=[{'fixture_roll_deg': LaunchConfiguration('fixture_roll_deg')}],
        ),

        # Real vehicle only: serial link to the docking mechanism Arduino.
        # In sim, abv_simulator stands in for the mechanism.
        Node(
            package='abv_dock_bridge',
            executable='dock_bridge',
            name='abv_dock_bridge',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration('sim')),
        ),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('sim')),
            actions=[
                Node(package='abv_simulator', executable='abv_simulator'),
                Node(package='abv_gui', executable='abv_gui'),
                Node(package='abv_commander', executable='abv_commander'),
            ]
        )
    ])
