import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class cmdVelPub(Node):
	def __init__(self):
		super().__init__("cmd_vel_pub_py")
		# QoS for Control commands: erliable, keeping last 10
		qos = QoSProfile(history = HistoryPolicy.KEEP_LAST, depth = 10)
		qos.reliability = ReliabilityPolicy.RELIABLE
		self.pub = self.create_publisher(Twist, 'cmd_vel', qos_profile = qos)
		self.timer = self.create_timer(0.1, self.publish_cmd) # 10Hz
		self.step = 0
		
	def publish_cmd(self):
		t = Twist()
		
		#simple sequence: forward, rotate. stop
		if self.step < 500:
			t.linear.x = 0.15
			t.angular.z = 0.0
		elif self.step < 1000:
			t.linear.x = 0.0
			t.angular.z = 0.5
		else:
			t.linear.x = 0.0
			t.angular.z = 0.0
		
		self.pub.publish(t)
		self.get_logger().info(f"pub cmd_vel: lin: {t.linear.x:.2f}, ang: {t.angular.z:.2f}")
		self.step += 1
		
def main(args = None):
	rclpy.init(args = args)
	node = cmdVelPub()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
			
	node.destroy_node()
	rclpy.shutdown()
		
if __name__ == '__main__':
    main()
