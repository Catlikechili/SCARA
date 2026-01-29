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
    
    # 1. Parse URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'myrobot.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # 2. Gazebo Path
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_share, '..')

    # 3. Chạy Gazebo có GUI nhưng dùng cấu hình tối giản
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'gui': 'true'}.items() 
    )

    # 4. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # 5. Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'myrobot'],
        output='screen'
    )

    # 6. Spawners
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