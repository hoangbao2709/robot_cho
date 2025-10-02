from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = FindPackageShare('yahboom_dog_description')

    # URDF bằng xacro

    urdf_file = PathJoinSubstitution([pkg_path, "urdf", "yahboom_dog.urdf.xacro"])
    robot_description = {
        "robot_description": Command([
            "xacro ",
            urdf_file
        ])
    }



    # File cấu hình controllers
    control_params = PathJoinSubstitution([pkg_path, 'config', 'controllers.yaml'])

    # Node ros2_control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, control_params],
        output='screen'
    )

    # Node robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"use_sim_time": True}, robot_description],
        output='screen'
    )

    # Gazebo (môi trường trống)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ])
    )

    # Spawn robot vào Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'yahboom_dog'],
        output='screen'
    )

    # Spawner cho controller
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_legs = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['legs_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        gazebo,
        spawn_robot,
        load_jsb,
        load_legs
    ])

