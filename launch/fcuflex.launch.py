from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare arguments
    fcuflex_host_arg = DeclareLaunchArgument(
        'fcuflex_host',
        default_value='192.168.0.12',
        description='IP address of the FCUFLEX controller'
    )
    
    fcuflex_port_arg = DeclareLaunchArgument(
        'fcuflex_port',
        default_value='24',
        description='TCP port of the FCUFLEX controller'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Rate at which to update device status (Hz)'
    )
    
    # Create fcuflex node
    fcuflex_node = Node(
        package='fcuflex_ros2',
        executable='fcuflex_node',
        name='fcuflex_node',
        parameters=[{
            'fcuflex_host': LaunchConfiguration('fcuflex_host'),
            'fcuflex_port': LaunchConfiguration('fcuflex_port'),
            'update_rate': LaunchConfiguration('update_rate')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        fcuflex_host_arg,
        fcuflex_port_arg,
        update_rate_arg,
        fcuflex_node
    ])