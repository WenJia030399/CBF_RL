#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageRecorder(Node):
    def __init__(self, topic='/tello1/image_raw', output_file='output.mp4', fps=30):
        super().__init__('image_recorder')
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.out = None
        self.output_file = output_file
        self.fps = fps
        self.frame_size = None
        self.recording_started = False
        self.frame_count = 0

        # 建立存影片的資料夾
        folder = os.path.dirname(output_file)
        if folder != '' and not os.path.exists(folder):
            os.makedirs(folder)

        self.get_logger().info(f"Subscribed to {topic}. Recording to {output_file}")

    def image_callback(self, msg):
        # 將 ROS Image 轉成 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not self.recording_started:
            height, width, _ = cv_image.shape
            self.frame_size = (width, height)
            # 使用 mp4 編碼
            self.out = cv2.VideoWriter(self.output_file, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, self.frame_size)
            self.recording_started = True
            self.get_logger().info(f"Video writer initialized: size={self.frame_size}, fps={self.fps}")

        # 寫入影片
        self.out.write(cv_image)
        self.frame_count += 1

        # optional: 顯示影像
        cv2.imshow("Recording", cv_image)
        cv2.waitKey(1)  # 1 ms

    def destroy_node(self):
        super().destroy_node()
        if self.out:
            self.out.release()
        cv2.destroyAllWindows()
        self.get_logger().info(f"Recording finished. Total frames: {self.frame_count}")

def main(args=None):
    rclpy.init(args=args)
    recorder = ImageRecorder(topic='/tello1/image_raw', output_file='tello_record.mp4', fps=30)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("KeyboardInterrupt, stopping recording...")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
