import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import degrees
from transforms3d.euler import quat2euler  # instead of tf_transformations


class OdomSub(Node):
    def __init__(self):
        super().__init__('odom_sub_py')
        self.sub = self.create_subscription(
            Odometry,
            '/odom',  # fixed topic name (no './')
            self.odom_callback,
            10  # QoS depth
        )
        self.get_logger().info("Odom Subscriber Node started.")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Convert quaternion -> euler angles (returns radians)
        q = [ori.x, ori.y, ori.z, ori.w]
        roll, pitch, yaw = quat2euler(q)

        self.get_logger().info(
            f"Position: x: {pos.x:.3f}, y: {pos.y:.3f} | "
            f"Orientation (yaw): {degrees(yaw):.1f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

