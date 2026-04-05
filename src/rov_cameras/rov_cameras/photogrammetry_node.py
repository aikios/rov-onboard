"""
Photogrammetry + camera preview node.

Runs on Pi 5. Two functions:
1. Preview: reads MJPEG stream from Pi Zero, publishes CompressedImage at ~5fps
2. Capture: on service call, triggers full-res capture for photogrammetry

Topics:
  /photogrammetry/preview (sensor_msgs/CompressedImage) — low-res preview stream
  /photogrammetry/image   (sensor_msgs/CompressedImage) — full-res captures

Service:
  /photogrammetry/capture (std_srvs/Trigger)
"""

import urllib.request
import threading
import io

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage


class PhotogrammetryNode(Node):
    def __init__(self):
        super().__init__('photogrammetry_node')

        self.declare_parameter('capture_url', 'http://192.168.7.2:8080/capture')
        self.declare_parameter('stream_url', 'http://192.168.7.2:8080/stream')
        self.declare_parameter('jpeg_quality', 95)

        self.capture_url = self.get_parameter('capture_url').value
        self.stream_url = self.get_parameter('stream_url').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.image_pub = self.create_publisher(
            CompressedImage, '/photogrammetry/image', 10)
        self.preview_pub = self.create_publisher(
            CompressedImage, '/photogrammetry/preview', 1)

        self.capture_srv = self.create_service(
            Trigger, '/photogrammetry/capture', self.capture_callback)

        self.capture_count = 0

        # Start MJPEG stream reader in background
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()

        self.get_logger().info(
            f'Photogrammetry node ready. Stream: {self.stream_url}')

    def _stream_loop(self):
        """Background: read MJPEG stream, publish preview frames."""
        import time
        while True:
            try:
                req = urllib.request.Request(self.stream_url)
                with urllib.request.urlopen(req, timeout=10) as resp:
                    boundary = b'--frame'
                    buf = b''
                    while True:
                        chunk = resp.read(4096)
                        if not chunk:
                            break
                        buf += chunk

                        # Find complete JPEG frames between boundaries
                        while boundary in buf:
                            idx = buf.find(boundary)
                            frame_data = buf[:idx]
                            buf = buf[idx + len(boundary):]

                            # Extract JPEG from the MIME part
                            jpeg_start = frame_data.find(b'\xff\xd8')
                            jpeg_end = frame_data.rfind(b'\xff\xd9')
                            if jpeg_start >= 0 and jpeg_end > jpeg_start:
                                jpeg = frame_data[jpeg_start:jpeg_end + 2]

                                msg = CompressedImage()
                                msg.header.stamp = self.get_clock().now().to_msg()
                                msg.header.frame_id = 'camera_preview'
                                msg.format = 'jpeg'
                                msg.data = list(jpeg)
                                self.preview_pub.publish(msg)

            except Exception as e:
                self.get_logger().warn(f'Stream error: {e}, reconnecting...')
                import time
                time.sleep(2)

    def capture_callback(self, request, response):
        """Full-res photogrammetry capture."""
        url = f'{self.capture_url}?quality={self.jpeg_quality}'
        self.get_logger().info(f'Capturing: {url}')

        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=15) as resp:
                image_data = resp.read()
                content_type = resp.headers.get('Content-Type', '')

            if 'jpeg' not in content_type and 'jpg' not in content_type:
                response.success = False
                response.message = f'Unexpected content type: {content_type}'
                return response

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'photogrammetry_camera'
            msg.format = 'jpeg'
            msg.data = list(image_data)
            self.image_pub.publish(msg)
            self.capture_count += 1

            response.success = True
            response.message = f'Capture #{self.capture_count}, {len(image_data)} bytes'
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Capture failed: {e}'
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
