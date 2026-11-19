"""Save camera frames as JPG files (for labeling and for testing the detector).

Usage:   ros2 run slalom save_images <folder> <every_nth_frame>
Example: ros2 run slalom save_images /home/projects/bilder/run1 3
Stop with Ctrl+C.
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def to_bgr(msg):
    """Convert a ROS Image (bgr8 or rgb8) to an OpenCV BGR array without cv_bridge."""
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


class Saver(Node):
    def __init__(self, folder, every):
        super().__init__('image_saver')
        self.folder, self.every = folder, every
        self.count, self.saved = 0, 0
        os.makedirs(folder, exist_ok=True)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image, 10)
        self.get_logger().info(f'Saving every {every}. frame to {folder} (Ctrl+C to stop)')

    def on_image(self, msg):
        self.count += 1
        if self.count % self.every:
            return
        name = os.path.join(self.folder, f'img_{int(time.time() * 1000)}.jpg')
        cv2.imwrite(name, to_bgr(msg), [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.saved += 1
        if self.saved % 10 == 0:
            self.get_logger().info(f'{self.saved} images saved')


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else 'bilder'
    every = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    rclpy.init()
    node = Saver(folder, every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(f'Done: {node.saved} images in {folder}')


if __name__ == '__main__':
    main()
