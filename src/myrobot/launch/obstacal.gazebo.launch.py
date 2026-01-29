import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'myrobot'
    pkg_share = get_package_share_directory(package_name)
    
    # 1. Parse URDF từ file Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'myrobot.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # 2. Cấu hình đường dẫn
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_share, '..')
    # Đường dẫn tới file world chứa tường
    world_file_path = os.path.join(pkg_share, 'worlds', 'my_world')

    # 3. Chạy Gazebo và load file World đã fix lỗi cú pháp
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'gui': 'true',
            'world': world_file_path
        }.items() 
    )

    # 4. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # 5. Spawn Robot vào môi trường
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'myrobot',
            
        ],
        output='screen'
    )

    # 6. Spawners (Bộ điều khiển)
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    
    load_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scara_controller"],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        
        # Event Handlers: Đợi robot spawn xong mới load controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_jtc],
            )
        ),
    ])