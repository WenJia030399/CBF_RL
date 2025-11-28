from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression # 🚨 新增 PythonExpression
from launch.conditions import IfCondition # 🚨 新增 IfCondition
import os

def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(os.getcwd(), 'uav_world.sdf'),
        description='Full path to the world file'
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch gzclient GUI?'
    )

    world_file = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    # 1. 啟動 Gazebo 伺服器 (Gzserver)
    gzserver_proc = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    # 2. 僅在 gui:=true 時啟動 Gazebo 客戶端 (Gzclient)
    gzclient_proc = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # 關鍵：使用條件式啟動
        condition=IfCondition(PythonExpression(['"', gui, '" == "true"'])) 
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        gzserver_proc, # 伺服器永遠啟動
        gzclient_proc, # 客戶端有條件啟動
    ])