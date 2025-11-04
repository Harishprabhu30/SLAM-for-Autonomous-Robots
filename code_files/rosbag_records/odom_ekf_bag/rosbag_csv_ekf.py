import rosbag2_py
import csv
import rclpy.serialization
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

bag_path = "/home/vgtu/Downloads/Harish_Thesis/ros2_humble/odom_ekf_bag"  # your bag folder
output_csv = "odom_ekf.csv"

storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr'
)
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {t.name: t.type for t in topic_types}

with open(output_csv, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        'timestamp (ns)', 'x', 'y', 'z',
        'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
        'linear_vel_x', 'linear_vel_y', 'linear_vel_z',
        'angular_vel_x', 'angular_vel_y', 'angular_vel_z'
    ])

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/odometry/filtered':
            msg = rclpy.serialization.deserialize_message(data, Odometry)
            p = msg.pose.pose
            tw = msg.twist.twist
            writer.writerow([
                t,
                p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
                tw.linear.x, tw.linear.y, tw.linear.z,
                tw.angular.x, tw.angular.y, tw.angular.z
            ])

print(f"✅ Exported /odom messages to {output_csv}")
