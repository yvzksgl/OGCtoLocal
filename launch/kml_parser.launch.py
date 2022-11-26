from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions.find_package import FindPackageShare

def generate_launch_description():
    rviz_config=FindPackageShare.find(FindPackageShare,"kml_parser")+"/launch/config/config.rviz"
    return LaunchDescription([
        Node(
            package='kml_parser',
            executable='kml_parser',
            name='kml_parser',
            parameters=[{"kml_file_path":"data/LocalizationAssignmentTestRoute.kml"}]
        ),
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config])
    ])