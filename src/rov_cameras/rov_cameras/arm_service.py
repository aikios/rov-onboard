"""
Arm/disarm service node.

Provides /rov/arm (std_srvs/SetBool) that arms/disarms the FC via pymavlink
through the MAVROS GCS UDP forward port.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from pymavlink import mavutil


class ArmService(Node):
    def __init__(self):
        super().__init__('arm_service')
        self.srv = self.create_service(SetBool, '/rov/arm', self.arm_callback)
        self.mav = None
        self.get_logger().info('Arm service ready on /rov/arm')

    def _connect(self):
        if self.mav is None:
            try:
                self.mav = mavutil.mavlink_connection(
                    'udpout:127.0.0.1:14560', source_system=255)
                self.mav.wait_heartbeat(timeout=3)
                self.get_logger().info('Connected to FC via GCS UDP')
            except Exception as e:
                self.get_logger().error(f'MAVLink connect failed: {e}')
                self.mav = None

    def arm_callback(self, request, response):
        self._connect()
        if self.mav is None:
            response.success = False
            response.message = 'No MAVLink connection'
            return response

        try:
            if request.data:
                self.mav.arducopter_arm()
                action = 'Armed'
            else:
                self.mav.arducopter_disarm()
                action = 'Disarmed'

            import time
            time.sleep(1)
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg:
                armed = bool(msg.base_mode & 0x80)
                response.success = armed == request.data
                response.message = f'{action} (FC reports armed={armed})'
            else:
                response.success = False
                response.message = f'{action} sent, no heartbeat to confirm'

        except Exception as e:
            response.success = False
            response.message = str(e)

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArmService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
