# -*- coding: utf-8 -*-
"""
Copyright (c) 2023 The uos_sess6072_build Authors.
All rights reserved.
Licensed under the BSD 3-Clause License.
See LICENSE.md file in the project root for full license information.
"""

import json
import time
import os

import numpy as np
from zeroros import MessageBroker, Publisher, Subscriber
from zeroros.messages import String, Twist, Vector3

from uos_sess6072_build.drivers import ThrustersDriver
from uos_sess6072_build.drivers.sense_hat_driver import (
    SenseHatDriver,
    PixelMatrix,
    RGBColor,
)
from uos_sess6072_build.tools import Console, Rate

"""
WARNING - This is the code that runs on the robot.
It is not meant to be run on the laptop.

DO NOT MODIFY THIS FILE.

It will have no effect on the robot.
"""


class Robot:
    def __init__(self):
        Console.set_logging_file("/home/robot/logs", name="robot")
        self.broker = MessageBroker(ip="*")
        self.configured = False
        self.config_sub = Subscriber("/config", String, self.config_cb)
        self.shutdown_sub = Subscriber("/shutdown", String, self.shutdown_cb)
        self.reboot_sub = Subscriber("/reboot", String, self.reboot_cb)
        self.rate = 5.0
        self.sense_hat = SenseHatDriver()
        self.sense_hat_matrix = PixelMatrix()
        self.thrusters = None
        self.count = 0
        self.stop = False
        while not self.configured and not self.stop:
            try:
                Console.info("Waiting for configuration on topic /config ...")
                if self.count % 2 == 0:
                    self.sense_hat_matrix.set_pixel(3, 3, RGBColor.RED)
                    self.sense_hat_matrix.set_pixel(4, 3, RGBColor.RED)
                    self.sense_hat_matrix.set_pixel(3, 4, RGBColor.RED)
                    self.sense_hat_matrix.set_pixel(4, 4, RGBColor.RED)
                    self.sense_hat.print_pixel_list(self.sense_hat_matrix.get())
                else:
                    self.sense_hat_matrix.set_pixel(3, 3, RGBColor.WHITE)
                    self.sense_hat_matrix.set_pixel(4, 3, RGBColor.WHITE)
                    self.sense_hat_matrix.set_pixel(3, 4, RGBColor.WHITE)
                    self.sense_hat_matrix.set_pixel(4, 4, RGBColor.WHITE)
                    self.sense_hat.print_pixel_list(self.sense_hat_matrix.get())
                time.sleep(1.0)
                self.count += 1
            except KeyboardInterrupt:
                Console.info("Ctrl+C pressed. Stopping...")
                self.config_sub.stop()
                self.shutdown_sub.stop()
                self.broker.stop()
                return

        # Reset LEDs
        self.sense_hat_matrix = PixelMatrix()
        self.sense_hat.print_pixel_list(self.sense_hat_matrix.get())

        Console.info("Starting robot...")
        r = Rate(self.rate)
        while not self.stop:
            try:
                self.loop()
            except KeyboardInterrupt:
                Console.info("Ctrl+C pressed. Stopping...")
                break
            r.sleep()
        self.config_sub.stop()
        self.shutdown_sub.stop()
        self.broker.stop()

    def __del__(self):
        if self.thrusters is not None:
            self.thrusters.move(0, 0)
        self.sense_hat_matrix = PixelMatrix()
        self.sense_hat.print_pixel_list(self.sense_hat_matrix.get())

    def config_cb(self, msg: String):
        Console.info("Configuration received!")
        json_data = msg.data
        params = json.loads(json_data)

        self.rate = params.get("rate", 5.0)
        left_port = params["left_port"]
        right_port = params["right_port"]
        tam = np.array(params["tam"]).reshape((2, 2))

        self.thrusters = ThrustersDriver(parent=self)
        success = self.thrusters.init(left_port, right_port, tam)
        if not success:
            Console.info(
                "Could not initialise the Pololu Micro Maestro. Is it connected?"
            )
            return
        self.imu_pub = Publisher("/imu", Vector3)
        self.cmd_vel_sub = Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        self.sense_hat.print_pixel_list(self.sense_hat_matrix.get())
        self.configured = True

    def loop(self):
        roll, pitch, yaw = None, None, None
        if not self.configured:
            return
        roll, pitch, yaw = self.sense_hat.read()
        self.imu_pub.publish(Vector3(roll, pitch, yaw))
        self.sense_hat_matrix.set_pixel(self.count % 8, 0, RGBColor.BLUE)
        self.sense_hat_matrix.set_pixel((self.count + 1) % 8, 0, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 2) % 8, 0, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 3) % 8, 0, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 4) % 8, 0, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 5) % 8, 0, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 6) % 8, 0, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 7) % 8, 0, RGBColor.BLACK)
        self.sense_hat.print_pixel_list(self.sense_hat_matrix.get())

    def cmd_vel_cb(self, msg: Twist):
        Console.info("Received cmd_vel message (x, z):", msg.linear.x, msg.angular.z)
        self.thrusters.move(msg.linear.x, msg.angular.z)
        self.sense_hat_matrix.set_pixel(self.count % 8, 1, RGBColor.GREEN)
        self.sense_hat_matrix.set_pixel((self.count + 1) % 8, 1, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 2) % 8, 1, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 3) % 8, 1, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 4) % 8, 1, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 5) % 8, 1, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 6) % 8, 1, RGBColor.BLACK)
        self.sense_hat_matrix.set_pixel((self.count + 7) % 8, 1, RGBColor.BLACK)
        self.sense_hat.print_pixel_list(self.sense_hat_matrix.get())
        self.count += 1

    def shutdown_cb(self, msg: String):
        self.configured = False
        self.sense_hat.print("Shutdown...")
        self.sense_hat_matrix = PixelMatrix()
        self.sense_hat.print_pixel_list(self.sense_hat_matrix.get())
        os.system("sudo shutdown -h now")
        self.stop = True

    def reboot_cb(self, msg: String):
        self.configured = False
        self.sense_hat.print("Reboot...")
        self.sense_hat_matrix = PixelMatrix()
        self.sense_hat.print_pixel_list(self.sense_hat_matrix.get())
        os.system("sudo reboot")
        self.stop = True


def main():
    r = Robot()
    del r


if __name__ == "__main__":
    main()
