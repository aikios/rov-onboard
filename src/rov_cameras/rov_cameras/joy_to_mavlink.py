"""
Joystick to MAVLink translation node.

Runs on Pi 5 (onboard). Subscribes to /joy from the topside DS4 controller
and publishes RC override commands to MAVROS for ArduSub control.

ArduSub RC Channel Mapping (Manual/Stabilize mode):
  CH1: Pitch         (forward/back body tilt)
  CH2: Roll          (left/right body tilt)
  CH3: Throttle      (vertical — up/down)
  CH4: Yaw           (rotate left/right)
  CH5: Forward       (forward/reverse thrust)
  CH6: Lateral       (strafe left/right)
  CH7: reserved
  CH8: Camera tilt / lights (configurable)

DS4 Axis Mapping (joy_linux):
  0: Left stick X  (left=1.0, right=-1.0)
  1: Left stick Y  (up=1.0, down=-1.0)
  2: L2 trigger    (released=1.0, pressed=-1.0)
  3: Right stick X (left=1.0, right=-1.0)
  4: Right stick Y (up=1.0, down=-1.0)
  5: R2 trigger    (released=1.0, pressed=-1.0)
  6: D-pad X       (left=1.0, right=-1.0)
  7: D-pad Y       (up=1.0, down=-1.0)

DS4 Button Mapping (joy_linux):
  0: Cross (X)      4: L1        8: Share    12: PS
  1: Circle (O)     5: R1        9: Options
  2: Triangle       6: L2        10: L3
  3: Square         7: R2        11: R3
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger


# ArduSub PWM
PWM_CENTER = 1500
PWM_RANGE = 400   # ±400 → 1100-1900
PWM_MIN = 1100
PWM_MAX = 1900
CHAN_NOCHANGE = 65535


def axis_to_pwm(value, deadzone=0.05):
    """Convert a -1.0..1.0 axis value to 1100-1900 PWM."""
    if abs(value) < deadzone:
        return PWM_CENTER
    clamped = max(-1.0, min(1.0, value))
    return int(max(PWM_MIN, min(PWM_MAX, PWM_CENTER + clamped * PWM_RANGE)))


def trigger_to_thrust(value):
    """Convert trigger axis (1.0=released, -1.0=pressed) to 0.0..1.0."""
    return (1.0 - value) / 2.0


class JoyToMavlink(Node):
    def __init__(self):
        super().__init__('joy_to_mavlink')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('enable_override', True)
        rate = self.get_parameter('publish_rate_hz').value
        self.enable_override = self.get_parameter('enable_override').value

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # RC Override publisher
        self.rc_pub = self.create_publisher(
            OverrideRCIn, '/mavros/mavros/override', 10
        )

        # Service clients for arm/disarm and mode set
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Photogrammetry capture client
        self.capture_client = self.create_client(Trigger, '/photogrammetry/capture')

        # Publish timer
        self.pub_timer = self.create_timer(1.0 / rate, self.publish_override)

        # Log timer (2 Hz for human readability)
        self.log_timer = self.create_timer(0.5, self.log_state)

        # State
        self.armed = False
        self.arm_button_prev = False
        self.capture_button_prev = False
        self.lights = PWM_CENTER  # 1100-1900

        # Channel values
        self.channels = [CHAN_NOCHANGE] * 18
        # Initialize controlled channels to center
        for i in range(6):
            self.channels[i] = PWM_CENTER

        self.last_joy_time = self.get_clock().now()
        self.joy_timeout_sec = 1.0  # safety: center if no joy for 1s

        self.get_logger().info(
            f'Joy→MAVLink node ready (override={self.enable_override}, rate={rate}Hz)'
        )

    def joy_callback(self, msg):
        if len(msg.axes) < 8 or len(msg.buttons) < 6:
            return

        self.last_joy_time = self.get_clock().now()

        # Map DS4 to ArduSub channels
        # CH5 (idx 4): Forward — left stick Y
        self.channels[4] = axis_to_pwm(msg.axes[1])
        # CH6 (idx 5): Lateral — right stick X
        self.channels[5] = axis_to_pwm(msg.axes[3])
        # CH4 (idx 3): Yaw — left stick X
        self.channels[3] = axis_to_pwm(msg.axes[0])
        # CH1 (idx 0): Pitch — right stick Y
        self.channels[0] = axis_to_pwm(msg.axes[4])

        # CH3 (idx 2): Throttle (vertical) — L2/R2 triggers
        ascend = trigger_to_thrust(msg.axes[5])   # R2
        descend = trigger_to_thrust(msg.axes[2])   # L2
        vertical = ascend - descend
        self.channels[2] = axis_to_pwm(vertical)

        # CH2 (idx 1): Roll — not mapped, center
        self.channels[1] = PWM_CENTER

        # Arm/Disarm on Cross (button 0) — edge detect
        cross_pressed = msg.buttons[0] == 1
        if cross_pressed and not self.arm_button_prev:
            self.toggle_arm()
        self.arm_button_prev = cross_pressed

        # Photogrammetry on Triangle (button 2) — edge detect
        tri_pressed = msg.buttons[2] == 1
        if tri_pressed and not self.capture_button_prev:
            self.trigger_capture()
        self.capture_button_prev = tri_pressed

        # Lights on L1/R1 (buttons 4/5)
        if msg.buttons[4] == 1:
            self.lights = max(PWM_MIN, self.lights - 50)
        if msg.buttons[5] == 1:
            self.lights = min(PWM_MAX, self.lights + 50)
        self.channels[7] = self.lights  # CH8 for lights

    def publish_override(self):
        # Safety: if no joy messages for > timeout, center everything
        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > self.joy_timeout_sec:
            for i in range(6):
                self.channels[i] = PWM_CENTER

        if not self.enable_override:
            return

        msg = OverrideRCIn()
        msg.channels = [int(c) for c in self.channels]
        self.rc_pub.publish(msg)

    def log_state(self):
        ch = self.channels
        self.get_logger().info(
            f'[{"ARMED" if self.armed else "DISARMED"}] '
            f'fwd={ch[4]} lat={ch[5]} vert={ch[2]} '
            f'yaw={ch[3]} pitch={ch[0]} lights={ch[7]}'
        )

    def toggle_arm(self):
        self.armed = not self.armed
        if not self.arm_client.service_is_ready():
            self.get_logger().warn('Arm service not available')
            return

        req = CommandBool.Request()
        req.value = self.armed
        future = self.arm_client.call_async(req)
        future.add_done_callback(self._arm_done)
        self.get_logger().info(f'{"Arming" if self.armed else "Disarming"}...')

    def _arm_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(
                    f'{"Armed" if self.armed else "Disarmed"} successfully'
                )
            else:
                self.get_logger().warn(f'Arm command failed: {result.result}')
                self.armed = not self.armed  # revert
        except Exception as e:
            self.get_logger().error(f'Arm service error: {e}')
            self.armed = not self.armed

    def trigger_capture(self):
        if not self.capture_client.service_is_ready():
            self.get_logger().warn('Photogrammetry capture service not available')
            return

        req = Trigger.Request()
        future = self.capture_client.call_async(req)
        future.add_done_callback(self._capture_done)
        self.get_logger().info('Photogrammetry capture triggered')

    def _capture_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'Capture: {result.message}')
            else:
                self.get_logger().warn(f'Capture failed: {result.message}')
        except Exception as e:
            self.get_logger().error(f'Capture service error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JoyToMavlink()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
