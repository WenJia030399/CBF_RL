from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to disable Gazebo GUI'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Full path to world file'
    )

    ld = LaunchDescription()
    ld.add_action(gui_arg)
    ld.add_action(world_arg)

    # gzserver
    ld.add_action(Node(
        package='gazebo_ros',
        executable='gzserver',
        output='screen',
        arguments=[LaunchConfiguration('world')]
    ))

    # gzclient (GUI)
    ld.add_action(Node(
        package='gazebo_ros',
        executable='gzclient',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
        arguments=[]  # 不使用 GUI plugin
    ))

    return ld
