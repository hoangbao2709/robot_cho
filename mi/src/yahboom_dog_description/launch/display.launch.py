from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os

def generate_launch_description():
    # Trỏ tới file Xacro trong src
    urdf_file = os.path.expanduser(
        '~/mi/src/yahboom_dog_description/urdf/yahboom_dog.urdf.xacro'
    )

    # Gọi xacro để generate URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

