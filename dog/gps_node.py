"""
gps_node.py
-----------
Optional Jetson-side GPS node for the u-blox SAM-M10Q connected directly
to the Jetson via UART.

NOTE: In the normal SASSA robot configuration the SAM-M10Q is wired to the
Teensy 4.1 (Serial2, RX=7/TX=8).  The Teensy publishes /fix via micro-ROS,
so this node is NOT started by dog_launch.py.

Run this node only if you have the SAM-M10Q wired directly to the Jetson:
  ros2 run dog gps_node --ros-args -p port:=/dev/ttyUSB0

Reads NMEA sentences (GGA) and publishes ROS2 NavSatFix messages.

Requires:
  pip install pyserial pynmea2

Published topics:
  /fix  (sensor_msgs/NavSatFix) — latitude, longitude, altitude

Parameters:
  port      (string)  — serial port  [default: GPS_PORT from robot_config]
  baud_rate (int)     — baud rate    [default: GPS_BAUD_RATE from robot_config]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from dog.robot_config import GPS_PORT, GPS_BAUD_RATE

try:
    import serial
    import pynmea2
    GPS_LIBS_AVAILABLE = True
except ImportError:
    GPS_LIBS_AVAILABLE = False

# Horizontal position variance for each fix type (m²).
# Based on u-blox SAM-M10Q datasheet CEP values converted to 1-sigma variance.
_COV_NO_FIX   = 9999.0   # unknown / no fix
_COV_GPS_FIX  =   4.0    # autonomous GNSS fix  (~2 m CEP → σ² ≈ 4 m²)
_COV_DGPS_FIX =   0.25   # DGPS / SBAS          (~0.5 m CEP → σ² ≈ 0.25 m²)
_COV_ALT      =   9.0    # altitude always ~3× worse than horizontal


class GpsNode(Node):

    def __init__(self):
        super().__init__('gps_node')

        self.declare_parameter('port',      GPS_PORT)
        self.declare_parameter('baud_rate', GPS_BAUD_RATE)

        port      = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self._ser    = None

        if not GPS_LIBS_AVAILABLE:
            self.get_logger().warn(
                'pyserial / pynmea2 not installed — GPS node inactive. '
                'Install with: pip install pyserial pynmea2'
            )
            return

        try:
            self._ser = serial.Serial(port, baudrate=baud_rate, timeout=1.0)
            self.get_logger().info(f'SAM-M10Q opened on {port} @ {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'GPS serial open failed: {e}')
            return

        # Read loop runs in a timer so the node stays responsive to shutdown.
        self.create_timer(0.0, self._read_sentence)

    def _read_sentence(self):
        if self._ser is None or not self._ser.is_open:
            return
        try:
            raw = self._ser.readline()
        except Exception as e:
            self.get_logger().error(f'GPS read error: {e}')
            return

        line = raw.decode('ascii', errors='ignore').strip()
        if not line.startswith('$'):
            return

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return

        if isinstance(msg, pynmea2.types.talker.GGA):
            self._publish_gga(msg)

    def _publish_gga(self, gga):
        fix = NavSatFix()
        fix.header.stamp    = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'

        quality = int(gga.gps_qual) if gga.gps_qual else 0

        if quality == 0:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            h_cov = _COV_NO_FIX
        elif quality >= 2:
            fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            h_cov = _COV_DGPS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_FIX
            h_cov = _COV_GPS_FIX

        fix.status.service = NavSatStatus.SERVICE_GPS

        try:
            fix.latitude  = gga.latitude  if gga.lat_dir == 'N' else -gga.latitude
            fix.longitude = gga.longitude if gga.lon_dir == 'E' else -gga.longitude
            fix.altitude  = float(gga.altitude)
        except (ValueError, TypeError):
            return

        # Diagonal position covariance [σ_lat², σ_lon², σ_alt²] (m²)
        fix.position_covariance = [
            h_cov, 0.0, 0.0,
            0.0, h_cov, 0.0,
            0.0, 0.0, _COV_ALT,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.fix_pub.publish(fix)

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
