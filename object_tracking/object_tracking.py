import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
import pyrealsense2 as rs

class ObjectTrackingPublisher(Node):
    def __init__(self):
        super().__init__('object_tracking_publisher')
        self.publisher_ = self.create_publisher(String, 'object_tracking', 10)
        # RealSenseカメラの設定
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

    def process_data(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        # 深度データの処理
        # ここに障害物検出ロジックを実装

        # 映像データの処理
        # ここに特定の色の服またはQRコードの検出ロジックを実装

        # 衝突回避と追跡の指示を生成
        command = "生成された指示"
        self.publish_command(command)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingPublisher()
    while rclpy.ok():
        node.process_data()
        rclpy.spin_once(node)
    node.pipeline.stop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

