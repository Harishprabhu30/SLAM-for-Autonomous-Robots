from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.actions import TimerAction

def generate_launch_description():
    # Find the package share folder (first jetbot_control/)
    pkg_share = FindPackageShare('jetbot_control').find('jetbot_control')

    # Correct path to ekf.yaml in config/ (do NOT add extra 'jetbot_control')
    ekf_config = PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])
    
    # Define EKF node separately
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],  # important for Isaac Sim
    )

    # Delay EKF startup by 3 seconds
    delayed_ekf = TimerAction(
        period=3.0,
        actions=[ekf_node]
    )

    return LaunchDescription([
        Node(
            package='jetbot_control',
            executable='cmd_vel_pub_py',
            name='cmd_vel_pub_py',
            output='screen',
        ),
        Node(
            package='jetbot_control',
            executable='odom_with_cov_pub_py',
            name='odom_with_cov_pub_py',
            output='screen',
        ),
        # Uncomment if you want camera node
        # Node(
        #     package='jetbot_control',
        #     executable='camera_sub_py',
        #     name='camera_sub_py',
        #     output='screen',
        # ),
        delayed_ekf,
    ])


	#self.get_logger().info(f"[{self.get_clock().now().to_msg().sec}] Jetbot x={pos.x:.2f}, y={pos.y:.2f}")


### using launch argument substitution:

#from launch import LaunchDescription
#from launch.actions import DeclareLaunchArgument
#from launch_ros.actions import Node
#from launch.substitutions import LaunchConfiguration

#def generate_launch_description():
#    log_level = LaunchConfiguration('log_level', default='info')
#    return LaunchDescription([
#        DeclareLaunchArgument('log_level', default_value='info'),
#        Node(
#            package='jetbot_control',
#            executable='jetbot_node',
#            output='screen',
#            arguments=['--ros-args', '--log-level', log_level]
#        )
#    ])


