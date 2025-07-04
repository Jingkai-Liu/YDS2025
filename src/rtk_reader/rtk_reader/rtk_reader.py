# File: rtk_reader/rtk_reader/rtk_reader_node.py

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial

class RtkInfo(Node):
    def __init__(self, name):
        super().__init__(name)
        self.yaw_cache = None
        self.publisher = self.create_publisher(Float64MultiArray, "/rtk_lla_info", 10)
        self.pub_clk = self.create_timer(0.1, self.read_serial)
#        self.serial = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.05)

    def read_serial(self):
        if self.serial.is_open:
            data = self.serial.readline().strip().decode("ascii")
            if data.startswith("$GPRMC"):
                parts = data.split(",")
                if len(parts) >= 9 and parts[2] == "A":
                    lat = float(parts[3][0:2]) + float(parts[3][2:]) / 60.0
                    lon = float(parts[5][0:3]) + float(parts[5][3:]) / 60.0
                    alt = 0
                    vel = float(parts[7]) * 1.852 / 3.6
                    vel_heading = float(parts[8])
                    yaw = 0 if self.yaw_cache is None else self.yaw_cache
                    msg = Float64MultiArray(data=[lon, lat, alt, vel, vel_heading, yaw])
                    self.publisher.publish(msg)
            elif data.startswith("#HEADING2A"):
                data2 = data.split(";")[1]
                if data2.startswith("SOL_COMPUTED") or data2.startswith("INTEGRITY_WARNING"):
                    parts = data2.split(",")
                    self.yaw_cache = float(parts[3])


def main():
    rclpy.init()
    rclpy.spin(RtkInfo(name="rtk_position"))
    rclpy.shutdown()

if __name__ == '__main__':
    main()