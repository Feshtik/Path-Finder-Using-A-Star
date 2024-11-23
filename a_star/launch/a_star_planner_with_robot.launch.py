from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory path for both packages
    a_star_share_dir = get_package_share_directory('a_star')
    innok_description_share_dir = get_package_share_directory('innok_description')

    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # URDF file for the robot from innok_description package
    innok_urdf_file = os.path.join(innok_description_share_dir, 'urdf', 'innok.urdf')

    # A* planner node
    a_star_node = Node(
        package='a_star',
        executable='a_star_node',
        name='a_star_node',
        output='screen',
        parameters=[
            {'robot_description': innok_urdf_file},  # Pass robot description as a parameter
            {'use_sim_time': use_sim_time}  # Use simulation time if True
        ]
    )

    # Robot state publisher from innok_description package
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': innok_urdf_file}]
    )

    # RViz for visualization with configuration file from innok_description package
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(innok_description_share_dir, 'config', 'display.rviz')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) time if true'
        ),
        a_star_node,
        robot_state_publisher_node,
        rviz_node
    ])