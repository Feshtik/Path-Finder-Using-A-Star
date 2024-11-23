from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a_star',
            executable='a_star_node',  # The name of your Python script without the file extension
            output='screen',  # Show output messages on the screen
            # Add any parameters or arguments your node requires here
        )
    ])