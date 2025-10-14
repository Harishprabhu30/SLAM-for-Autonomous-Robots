from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='jetbot_control',
			executable='cmd_vel_pub_py',
			name='cmd_vel_pub_py',
			output='screen',
			arguments=['--ros-args', '--log-level', 'INFO']
		),
		Node(
			package='jetbot_control',
			executable='odom_sub_py',
			name='odom_sub_py',
			output='screen',
			arguments=['--ros-args', '--log-level', 'INFO']
		),
		Node(
			package='jetbot_control',
			executable='camera_sub_py',
			name='camera_sub_py',
			output='screen',
			arguments=['--ros-args', '--log-level', 'INFO']
		)
	])

	#self.get_logger().info(f"[{self.get_clock().now().to_msg().sec}] Jetbot x={pos.x:.2f}, y={pos.y:.2f}")

