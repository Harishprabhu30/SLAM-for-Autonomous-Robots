#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile


class TFBroadcasterSim(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_sim')

        qos = QoSProfile(depth=10)
        self.br = TransformBroadcaster(self, qos=qos)
        self.timer = self.create_timer(0.05, self.broadcast_tf)  # 20 Hz

        self.get_logger().info("TF Broadcaster (Isaac Sim) started.")

    def broadcast_tf(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            # /clock not yet started → skip until sim time active
            self.get_logger().debug("Waiting for /clock to start...")
            return

        # World → Odom
        t1 = TransformStamped()
        t1.header.stamp = now.to_msg()
        t1.header.frame_id = "world"
        t1.child_frame_id = "odom"
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0

        # Odom → Chassis
        t2 = TransformStamped()
        t2.header.stamp = now.to_msg()
        t2.header.frame_id = "odom"
        t2.child_frame_id = "chassis"
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        # Chassis → Left Wheel
        t3 = TransformStamped()
        t3.header.stamp = now.to_msg()
        t3.header.frame_id = "chassis"
        t3.child_frame_id = "left_wheel"
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.093
        t3.transform.translation.z = 0.0
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0

        # Chassis → Right Wheel
        t4 = TransformStamped()
        t4.header.stamp = now.to_msg()
        t4.header.frame_id = "chassis"
        t4.child_frame_id = "right_wheel"
        t4.transform.translation.x = 0.0
        t4.transform.translation.y = -0.093
        t4.transform.translation.z = 0.0
        t4.transform.rotation.x = 0.0
        t4.transform.rotation.y = 0.0
        t4.transform.rotation.z = 0.0
        t4.transform.rotation.w = 1.0

        # Publish all transforms
        self.br.sendTransform([t1, t2, t3, t4])
        	       
        now_msg = now.to_msg()
        self.get_logger().debug(f"Published TF at t={now_msg.sec}.{now_msg.nanosec}")	
	# Log timestamp safely
	
	
def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

