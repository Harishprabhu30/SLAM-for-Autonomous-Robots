import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class cmdVelPub(Node):
    def __init__(self):
        super().__init__("cmd_vel_pub_py")
        # QoS for control commands: Best Effort, Volatile, Keep Last 10
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            #durability=DurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos_profile=qos)
        self.timer = self.create_timer(0.05, self.publish_cmd)  # 10Hz
        self.step = 0
        
        # Initial zero publish to establish connection without sudden movement
        initial_twist = Twist()
        self.pub.publish(initial_twist)
        self.get_logger().info("Initialized with zero cmd_vel for stable startup")

    def publish_cmd(self):
        t = Twist()
        
        # Simple sequence: forward, rotate, forward, stop
        if self.step < 400:
            t.linear.x = 0.10
            t.angular.z = 0.0
        elif self.step < 600:
            t.linear.x = 0.0
            t.angular.z = 0.2
        elif self.step < 800:
            t.linear.x = 0.10
            t.angular.z = 0.0
        else:
            t.linear.x = 0.0
            t.angular.z = 0.0
        
        self.pub.publish(t)
        self.get_logger().info(f"pub cmd_vel: lin: {t.linear.x:.2f}, ang: {t.angular.z:.2f}")
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = cmdVelPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
