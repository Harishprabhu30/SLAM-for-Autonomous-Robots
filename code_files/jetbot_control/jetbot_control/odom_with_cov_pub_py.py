print("Develop branch version")

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile

class OdomWithCovPublisher(Node):
    def __init__(self):
        super().__init__('odom_with_cov_publisher')

        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_callback, 
            qos
        )
        self.pub = self.create_publisher(
            Odometry, 
            '/odom_with_cov', 
            qos
        )

        self.get_logger().info("OdomWithCovPublisher started.")

    def odom_callback(self, msg: Odometry):
        # Set realistic covariance
        cov = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]
#        cov = [
#            0.01, 0, 0, 0, 0, 0,
#            0, 0.01, 0, 0, 0, 0,
#            0, 0, 0.01, 0, 0, 0,
#            0, 0, 0, 0.01, 0, 0,
#            0, 0, 0, 0, 0.01, 0,
#           0, 0, 0, 0, 0, 0.01
#        ]
#        cov = [0.01 for _ in range(36)]
 
        msg.pose.covariance = cov
        msg.twist.covariance = cov

        # Ensure child_frame_id matches EKF config
        msg.child_frame_id = "chassis"

        # Publish with covariance
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomWithCovPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

