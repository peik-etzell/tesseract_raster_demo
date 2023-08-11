from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import load_xacro
from launch_param_builder.utils import get_package_share_directory
from pathlib import Path


def generate_launch_description():

    robot_files_dir = Path(
        get_package_share_directory('tesseract_support'),
        'urdf'
    )
    robot_name = 'lbr_iiwa_14_r820'
    robot_description = load_xacro(
        robot_files_dir / Path(f'{robot_name}.xacro'))
    robot_description_semantic = load_xacro(
        robot_files_dir / Path(f'{robot_name}.srdf'))

    demo_node = Node(
        package="tesseract_demo",
        executable="tesseract_demo_node",
        # prefix=['gdbserver localhost:1234'],
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "debug": True
            }
        ],
    )

    return LaunchDescription([demo_node])
