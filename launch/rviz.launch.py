from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder.utils import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from os import path


def generate_launch_description():

    tf2_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="virtual_joint_broadcaster_0",
        output="screen",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link"
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', path.join(
            get_package_share_directory("tesseract_ros_examples"),
            'config',
            'examples.rviz'
        )],
    )

    return LaunchDescription([tf2_node, rviz_node])
