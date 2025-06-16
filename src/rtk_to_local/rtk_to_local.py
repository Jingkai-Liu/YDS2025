#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

from veh_msgs.srv import SetLocalOrigin


def lla2ecef(lon, lat, alt):
    """
    unit of inputs should be rad
    """
    a = 6378137.0  # a axis
    f = 1 / 298.257223565  # f = (a-b)/a
    e_square = f * (2 - f)
    N = a / math.sqrt(1 - e_square * math.sin(lat) ** 2)
    X = (N + alt) * math.cos(lat) * math.cos(lon)
    Y = (N + alt) * math.cos(lat) * math.sin(lon)
    Z = (N * (1 - e_square) + alt) * math.sin(lat)

    return [X, Y, Z]


def lla2enu(lon, lat, alt, lon0, lat0, alt0):
    p0 = lla2ecef(lon0, lat0, alt0)
    p = lla2ecef(lon, lat, alt)
    [u, v, w] = [p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]]
    cos_lon0 = math.cos(lon0)
    sin_lon0 = math.sin(lon0)
    cos_lat0 = math.cos(lat0)
    sin_lat0 = math.sin(lat0)

    e = -sin_lon0 * u + cos_lon0 * v
    t = cos_lon0 * u + sin_lon0 * v

    n = -sin_lat0 * t + cos_lat0 * w
    u = cos_lat0 * t + sin_lat0 * w
    # if e << 1, there is the approximation:
    # e = a * math.cos(lat)*delta_lon
    # n = a * delta_lat
    # u = delta_alt
    return [e, n, u]


def deg2rad(x):
    return x * math.pi / 180.0


class RtkInfo(Node):
    def __init__(self, name):
        super().__init__(name)
        self.origin = None
        self.enu = None
        self.vel_yaw = None

        self.subscriber = self.create_subscription(
            Float64MultiArray, "/rtk_lla_info", self.get_rkt_info, rclpy.qos.qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(Float64MultiArray, "/local_info", 1)
        self.pub_clk = self.create_timer(0.2, self.pub_pose)

        self.set_origin_srv = self.create_service(SetLocalOrigin, "set_origin", self.set_local_origin)

    def set_local_origin(self, request, response):
        self.origin = [request.origin_lon, request.origin_lat, request.origin_alt]
        self.get_logger().info(f"set local coordination origin at {self.origin}")
        response.ok = True
        return response

    def get_rkt_info(self, msg):
        data = msg.data
        lon = deg2rad(data[0])
        lat = deg2rad(data[1])
        alt = deg2rad(data[2])

        vel = data[3]
        vel_heading = data[4]
        vel_heading = deg2rad(((360.0 - vel_heading) + 90) % 360.0)

        yaw = data[5]
        yaw = deg2rad(((360.0 - yaw) + 90) % 360.0)

        self.vel_yaw = [vel, vel_heading, yaw]

        if self.origin is None:
            self.enu = None
        else:
            self.enu = lla2enu(
                lon,
                lat,
                alt,
                deg2rad(self.origin[0]),
                deg2rad(self.origin[1]),
                deg2rad(self.origin[2]),
            )
        return

    def pub_pose(self):
        if self.enu is None:
            return
        else:
            msg = Float64MultiArray(data=self.enu + self.vel_yaw)
            self.publisher.publish(msg)
        return


def main():
    rclpy.init()
    rclpy.spin(RtkInfo(name="calc_local_position"))
    rclpy.shutdown()
