#!/usr/bin/env python3

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

        msg.pose.covariance = cov
        msg.twist.covariance = cov

        # Get current simulation time
        now = self.get_clock().now()

        # Stamp with sim time and correct frames
        #msg.header.stamp = now.to_msg()
        self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"      # must match EKF odom_frame
        msg.child_frame_id = "chassis"    # must match EKF base_link_frame

        # Publish updated odometry
        self.pub.publish(msg)

        # ✅ Correct way to log current time
        self.get_logger().debug(
            f"Published odom_with_cov at t={now.seconds_nanoseconds()[0]}.{now.seconds_nanoseconds()[1]}"
        )


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

