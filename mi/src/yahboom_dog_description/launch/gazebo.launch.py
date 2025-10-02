from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare   # <-- thiếu cái này
from ament_index_python.packages import get_package_share_directory  # <-- thêm cho ros2_control_node
import os


def generate_launch_description():
    pkg_path = FindPackageShare('yahboom_dog_description')

    urdf_file = PathJoinSubstitution([pkg_path, "urdf", "yahboom_dog.urdf.xacro"])
    robot_description = {
        "robot_description": Command([
            "xacro ",
            urdf_file
        ])
    }

    ros2_controllers_path = os.path.join(
        get_package_share_directory("yahboom_dog_description"),
        "config",
        "controllers.yaml"
    )

    # Controller manager node (ros2_control_node)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"use_sim_time": True}, robot_description],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ])
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'yahboom_dog'],
        output='screen'
    )

    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_legs = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['legs_position_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,   # <-- thêm node này
        gazebo,
        spawn_robot,
        load_jsb,
        load_legs
    ])

