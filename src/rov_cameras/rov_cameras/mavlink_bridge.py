"""
MAVLink bridge — owns the serial port, forwards to UDP for MAVROS.

This process:
1. Connects to /dev/ttyACM0 (FC serial)
2. Forwards all FC messages to UDP 127.0.0.1:14550 (MAVROS reads this)
3. Listens on UDP 127.0.0.1:14560 for commands from joy_to_mavlink
4. Forwards those commands to the FC

This allows both MAVROS (read-only) and joy_to_mavlink (write) to
communicate with the FC without conflicts.
"""

import threading
import time
import socket
from pymavlink import mavutil

SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 115200
MAVROS_UDP_OUT = ('127.0.0.1', 14550)   # MAVROS reads from here
CMD_UDP_LISTEN = ('127.0.0.1', 14560)   # joy_to_mavlink sends commands here


def main():
    print(f'MAVLink bridge starting...')
    print(f'  Serial: {SERIAL_PORT} @ {SERIAL_BAUD}')
    print(f'  MAVROS UDP out: {MAVROS_UDP_OUT}')
    print(f'  Command UDP in: {CMD_UDP_LISTEN}')

    # Connect to FC serial
    fc = mavutil.mavlink_connection(SERIAL_PORT, baud=SERIAL_BAUD)
    fc.wait_heartbeat(timeout=10)
    print(f'  FC connected: system {fc.target_system}')

    # UDP socket for forwarding TO MAVROS
    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # UDP socket for receiving commands FROM joy_to_mavlink
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.bind(CMD_UDP_LISTEN)
    cmd_sock.setblocking(False)

    print('  Bridge running')

    def forward_fc_to_mavros():
        """Read from FC serial, forward raw bytes to MAVROS UDP."""
        while True:
            try:
                # Read raw bytes from serial
                data = fc.port.read(256)
                if data:
                    out_sock.sendto(data, MAVROS_UDP_OUT)
            except Exception as e:
                print(f'FC→MAVROS error: {e}')
                time.sleep(0.1)

    def forward_cmds_to_fc():
        """Read commands from UDP, forward raw bytes to FC serial."""
        while True:
            try:
                data, addr = cmd_sock.recvfrom(1024)
                if data:
                    fc.port.write(data)
            except BlockingIOError:
                time.sleep(0.005)
            except Exception as e:
                print(f'CMD→FC error: {e}')
                time.sleep(0.1)

    # Send heartbeat to keep FC alive
    def heartbeat_loop():
        while True:
            try:
                fc.mav.heartbeat_send(6, 8, 0, 0, 0)
            except:
                pass
            time.sleep(1)

    t1 = threading.Thread(target=forward_fc_to_mavros, daemon=True)
    t2 = threading.Thread(target=forward_cmds_to_fc, daemon=True)
    t3 = threading.Thread(target=heartbeat_loop, daemon=True)
    t1.start()
    t2.start()
    t3.start()

    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('Bridge stopped')


if __name__ == '__main__':
    main()
