import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSub(Node):
    def __init__(self):
        super().__init__('camera_sub_py')
        self.bridge = CvBridge()

        qos_profile = rclpy.qos.QoSProfile(
            reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )

        self.sub = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            qos_profile
        )

        self.get_logger().info("Camera Subscriber Started.")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Jetbot RGB', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
    	#node.cleanup()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

