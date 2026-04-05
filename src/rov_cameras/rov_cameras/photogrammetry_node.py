"""
Photogrammetry capture node.

Runs on Pi 5. Triggers the Pi Zero HTTP capture server and publishes
full-resolution images on a ROS2 topic for topside to receive/save.

Service: /photogrammetry/capture (std_srvs/Trigger)
Topic:   /photogrammetry/image (sensor_msgs/CompressedImage)
"""

import urllib.request
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage


class PhotogrammetryNode(Node):
    def __init__(self):
        super().__init__('photogrammetry_node')

        self.declare_parameter('capture_url', 'http://192.168.7.2:8080/capture')
        self.declare_parameter('status_url', 'http://192.168.7.2:8080/status')
        self.declare_parameter('jpeg_quality', 95)

        self.capture_url = self.get_parameter('capture_url').value
        self.status_url = self.get_parameter('status_url').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.image_pub = self.create_publisher(
            CompressedImage, '/photogrammetry/image', 10
        )

        self.capture_srv = self.create_service(
            Trigger, '/photogrammetry/capture', self.capture_callback
        )

        self.capture_count = 0
        self.get_logger().info(
            f'Photogrammetry node ready. Capture URL: {self.capture_url}'
        )

    def capture_callback(self, request, response):
        url = f'{self.capture_url}?quality={self.jpeg_quality}'
        self.get_logger().info(f'Triggering capture: {url}')

        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=10) as resp:
                image_data = resp.read()
                content_type = resp.headers.get('Content-Type', '')

            if 'jpeg' not in content_type and 'jpg' not in content_type:
                response.success = False
                response.message = f'Unexpected content type: {content_type}'
                self.get_logger().error(response.message)
                return response

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'photogrammetry_camera'
            msg.format = 'jpeg'
            msg.data = list(image_data)

            self.image_pub.publish(msg)
            self.capture_count += 1

            response.success = True
            response.message = (
                f'Capture #{self.capture_count}, '
                f'{len(image_data)} bytes'
            )
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Capture failed: {str(e)}'
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PhotogrammetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
