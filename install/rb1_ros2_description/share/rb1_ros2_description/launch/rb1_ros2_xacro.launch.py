import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import xacro


def generate_launch_description():

    # Define package and file paths
    description_package_name = "rb1_ros2_description"
    package_dir = get_package_share_directory(description_package_name)
    robot_desc_file = os.path.join(package_dir, 'xacro', 'rb1_ros2_base.urdf.xacro')

    controller_config_file = os.path.join(package_dir, 'config', 'rb1_controller.yaml')

    # Parse Xacro file
    try:
        doc = xacro.parse(open(robot_desc_file))
        xacro.process_doc(doc)
        robot_description_config = doc.toxml()
        print("Parsed robot description successfully.")
    except Exception as e:
        print(f"Error parsing Xacro file: {e}")
        raise

    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'verbose': 'false', 'pause': 'false'}.items(),
    )

    # Robot State Publisher Node
    rsp_robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
        ],
        output="screen"
    )

    # Controller Manager Node (ros2_control_node)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description_config},
            controller_config_file
        ],
        output="screen",
    )

    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rb1_robot',
            '-topic', '/robot_description'
        ],
        output="screen"
    )

    # Spawners for controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen"
    )

    # Event handlers to sequence the controller spawners
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    delay_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner]
        )
    )

    # Launch description
    return LaunchDescription([
        gazebo,
        rsp_robot,
        controller_manager,
        spawn_robot,
        delay_joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner
    ])


