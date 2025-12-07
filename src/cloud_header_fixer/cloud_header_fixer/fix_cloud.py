import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

class CloudHeaderFixer(Node):
    def __init__(self):
        super().__init__("cloud_header_fixer")

        # SUBSCRIBE to the corrected pointcloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            "/unilidar/points",
            self.callback,
            10
        )

        # PUBLISH fixed cloud on a new topic
        self.pub = self.create_publisher(
            PointCloud2,
            "/unilidar/points_fixed",
            10
        )

        self.get_logger().info("CloudHeaderFixer started.")

    def callback(self, msg):
        # Fix header
        new_header = Header()
        new_header.stamp = self.get_clock().now().to_msg()
        new_header.frame_id = "unilidar_lidar"

        msg.header = new_header
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CloudHeaderFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
