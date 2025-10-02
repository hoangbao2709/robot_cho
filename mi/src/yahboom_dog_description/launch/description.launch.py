import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('yahboom_dog_description').find('yahboom_dog_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'yahboom_dog.urdf.xacro')

    # chạy xacro để tạo urdf
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file
    ])

    # ép kiểu string cho ROS2
    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])

