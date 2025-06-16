#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

import math
import numpy as np
from scipy.io import loadmat
import os


class CalcError(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("data_path", "")

        filepath = self.get_parameter("data_path").get_parameter_value().string_value
        self.get_logger().info(f"get path `{filepath}`")
        self.path_data = self.read_path(filepath)

        self.path_sub = self.create_subscription(Float64MultiArray, "/target_path", self.path_recvived_callback, 1)

        self.subscriber = self.create_subscription(Float64MultiArray, "/local_info", self.pose_received_callback, 1)

        self.publisher = self.create_publisher(Float64MultiArray, "/car_err", 1)

    def read_path(self, filepath):
        if not (os.path.exists(filepath) and os.path.isfile(filepath)):
            self.get_logger().warning(f"empty data path")
            return None
        data = loadmat(filepath)
        return data["path"]

    def path_recvived_callback(self, msg):
        self.path_data = np.array(msg.data).reshape((5, -1))
        self.get_logger().info("recv path_data")

    def pose_received_callback(self, msg):
        if self.path_data is None:
            self.get_logger().warning("path data is empty")
            return
        pose = msg.data
        err = self.cal_error(pose)
        pub_msg = Float64MultiArray(data=err)
        self.publisher.publish(pub_msg)
        self.get_logger().debug(f"receive {msg.data}")
        self.get_logger().debug(f"publish {pub_msg.data}")

    def cal_error(self, pose):
        x = pose[0]
        y = pose[1]
        z = pose[2]
        vel = pose[3]
        vel_heading = pose[4]
        yaw = pose[5]
        s = self.path_data[0, :]
        xd = self.path_data[1, :]
        yd = self.path_data[2, :]
        theta_d = self.path_data[3, :]
        # kappa_d = self.path_data[4,:]

        idx = np.argmin((x - xd) ** 2 + (y - yd) ** 2)

        lat_err = -(x - xd[idx]) * np.sin(theta_d[idx]) + (y - yd[idx]) * np.cos(theta_d[idx])

        yaw_err = yaw - theta_d[idx]
        if yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        elif yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        if not (-math.pi <= yaw_err <= math.pi):
            self.get_logger().warning("yaw_err calc error")

        return [
            s[idx].item(),
            xd[idx].item(),
            yd[idx].item(),
            theta_d[idx].item(),
            lat_err.item(),
            yaw_err.item(),
        ]


def main(args=None):
    rclpy.init(args=args)
    node = CalcError("calc_error")  # "/home/rika/robotics_ws/src/spline/spline/path_data.mat"
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
