# -*- coding: utf-8 -*-
"""
Copyright (c) 2023 The uos_sess6072_build Authors.
All rights reserved.
Licensed under the BSD 3-Clause License.
See LICENSE.md file in the project root for full license information.
"""
import json
import time
import math

from zeroros import Publisher, Subscriber
from zeroros.messages import String, Twist, Vector3

from uos_sess6072_build.drivers import ArUcoUDPDriver
from uos_sess6072_build.tools import Console, Rate
from uos_sess6072_build import __version__

from pathlib import Path
from datetime import datetime

import numpy as np
from sess6072_tutorials import PID


class LaptopController:
    def __init__(self):
        # Declare robot IP address
        self.robot_ip = "192.168.10.1"
        self.robot_available = False

        Console.info("Starting laptop controller v" + __version__ + "")
        Console.info("Connecting to robot at", self.robot_ip, "")

        # Declare publishers and subscribers
        self.cmd_vel_pub = Publisher("/cmd_vel", Twist, ip=self.robot_ip)
        self.config_pub = Publisher("/config", String, ip=self.robot_ip)
        self.imu_sub = Subscriber("/imu", Vector3, self.imu_cb, ip=self.robot_ip)
        aruco_params = {
            "port": 50000,  # Port to listen to (DO NOT CHANGE)
            "marker_id": 21,  # Marker ID to listen to (CHANGE THIS to your marker ID)
        }
        self.aruco_driver = ArUcoUDPDriver(aruco_params, parent=self)

        # Declare variables
        self.sensed_imu_roll_rad = None
        self.sensed_imu_pitch_rad = None
        self.sensed_imu_yaw_rad = None
        self.sensed_imu_stamp_s = None
        self.sensed_pos_northings_m = None
        self.sensed_pos_eastings_m = None
        self.sensed_pos_yaw_rad = None
        self.sensed_pos_stamp_s = None

        init_MM_x = 0
        init_MM_y = 0
        init_MM_H = 0
        self.MMxs = [init_MM_x]
        self.MMys = [init_MM_y]
        self.MMHs = [init_MM_H]
        self.PIDTune = []

        self.prevyaw = 0
        self.prevnorth = 0
        self.preveast = 0
        self.prevt = time.time()
        self.prevheading = 0

        self.angular_speed_log = 0
        self.setpoint = 0

        self.stop_robot = False

        self.switch = 0
        self.q = 0

        self.my_var = None
        self.current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = Path('Test3_Aruco_' + self.current_time + '.csv')
        self.fn = Path("Test3_Log_" + self.current_time + '.csv')

        # Log ARUCO data to file
        with self.filename.open('w') as f:
            f.write('epoch,northings,eastings,yaw\n')

        # Log ASV data to file
        with self.fn.open('w') as f2:
            f2.write('epoch,x_MM,y_MM,heading_MM,linear_vel_request,angular_vel_request,roll,pitch,yaw,rpm_left,rpm_right,yaw_unwrapped\n')

        self.motor_start_time = None
      
        # Declare parameters to be sent to the robot
        rate = 5.0  # Hz
        r = Rate(rate)
        params = {
            "rate": rate,
            "left_port": 0,
            "right_port": 1,
            "tam": [[1500, 500], [-1500, 500]],
        }

        # Wait for robot to be available - this is a blocking call
        count = 0
        while not self.robot_available:
            if count % 10 == 0:
                Console.info("Waiting for robot to be available in the network...")
            self.config_pub.publish(String(json.dumps(params)))
            time.sleep(1.0)
            count += 1

        # Main loop
        Console.info("Robot ready. Starting the main loop...")
        while True:
            try:
                self.loop()
            except KeyboardInterrupt:
                Console.info("Ctrl+C pressed. Stopping...")
                self.imu_sub.stop()
                break
            r.sleep()

    def imu_cb(self, msg: Vector3):
        """This is the callback for the IMU subscriber.

        Parameters
        ----------
        msg : Vector3
            Orientation of the robot in roll, pitch, yaw.
        """
        self.sensed_imu_roll_rad = msg.x
        self.sensed_imu_pitch_rad = msg.y
        self.sensed_imu_yaw_rad = msg.z
        self.sensed_imu_stamp_s = time.time()
        self.robot_available = True

    def loop(self):
        """This is the main loop of the controller."""
        
        current_epoch_s = time.time()

        # > Sense < #
        # get the latest IMU and position measurements
        sensed_pos = self.aruco_driver.read()
        if sensed_pos is not None:
            self.sensed_pos_stamp_s = sensed_pos[0]
            self.sensed_pos_northings_m = sensed_pos[1]
            self.sensed_pos_eastings_m = sensed_pos[2]
            self.sensed_pos_yaw_rad = sensed_pos[6]


        with self.filename.open('w') as f:
            f.write(f'{self.sensed_pos_yaw_rad},{self.sensed_pos_northings_m},{self.sensed_pos_eastings_m},{self.sensed_pos_yaw_rad}\n')


        # > Think < #
        #  TODO: Implement your controller here                                        

        dt = time.time()-self.prevt
        self.prevt = time.time()


        twist_msg = Twist()
        if self.motor_start_time is None:
            self.motor_start_time = time.time()
        current_epoch_s = time.time()

        if self.switch==0:
            self.setpoint = self.sensed_imu_yaw_rad
            self.switch = 1
            self.chk = self.sensed_imu_yaw_rad
        
        yaw_change = self.sensed_imu_yaw_rad - self.chk
        self.chk = self.sensed_imu_yaw_rad
        self.YAW = self.sensed_imu_yaw_rad
        if yaw_change < -2:
            self.q += 1
            #yaw_change = (-np.pi - self.sensed_imu_yaw_rad) + (np.pi - self.YAW)
        if yaw_change > 2:
            self.q -= 1
            #yaw_change = (np.pi - self.sensed_imu_yaw_rad) + (-np.pi - self.YAW)
        # self.YAW += yaw_change
        self.YAW += self.q*2*np.pi


        print("Setpoint: ", self.setpoint)
        print("Yaw: ", self.sensed_imu_yaw_rad)
        print("YAW: ", self.YAW)

        linear_speed1 = 0.25

        if current_epoch_s - self.motor_start_time > 2 and current_epoch_s - self.motor_start_time < 2.1:
            self.setpoint = self.sensed_imu_yaw_rad + np.deg2rad(-90)

        # if current_epoch_s - self.motor_start_time > 10 and current_epoch_s - self.motor_start_time < 10.1:
        #     self.setpoint = self.sensed_imu_yaw_rad + np.deg2rad(90)

        # if current_epoch_s - self.motor_start_time > 10 and current_epoch_s - self.motor_start_time < 10.1:
        #     self.setpoint = self.sensed_imu_yaw_rad + np.deg2rad(90)

        angular_PID = PID(2,0.0,0.01, self.setpoint, limits=(-0.8,0.8))
        angular_speed1 = angular_PID(self.YAW)
        print(angular_speed1)
        # linear_speed1 = 0.25
        
        # print(np.round(angular_speed1, 3), 
        #       np.round(np.rad2deg(self.setpoint), 3),
        #       np.round(np.rad2deg(self.sensed_imu_yaw_rad), 3), 
        #       " | ", np.round((current_epoch_s - self.motor_start_time), 3))


        Crequest_linear_x_vel = linear_speed1
        Crequest_angular_z_vel = angular_speed1 

        if self.stop_robot:
            Crequest_linear_x_vel = 0.0
            Crequest_angular_z_vel = 0.0


        req = np.array([[Crequest_linear_x_vel], [Crequest_angular_z_vel]])
        rpm_ls = [[1500, 500], [-1500, 500]] @ req 

        rpm_left = abs(rpm_ls[0][0])
        rpm_right = abs(rpm_ls[1][0])
        print(rpm_left, rpm_right)

        self.PIDTune.append(Crequest_angular_z_vel)

        # Motion Model
        MMLspeed = Crequest_linear_x_vel
        MMAspeed = Crequest_angular_z_vel
        new_heading = self.MMHs[-1] + MMAspeed*dt
        distance = MMLspeed * dt

        dx = distance*np.sin(new_heading)
        dy = distance*np.cos(new_heading)
        self.MMxs.append(self.MMxs[-1]+dx)
        self.MMys.append(self.MMys[-1]+dy)
        self.MMHs.append(new_heading)

        with self.fn.open('a') as f2:
            f2.write(f"{current_epoch_s}, {self.MMxs[-1]}, {self.MMys[-1]}, {new_heading}, {Crequest_linear_x_vel}, {Crequest_angular_z_vel}, {self.sensed_imu_roll_rad}, {self.sensed_imu_pitch_rad}, {self.sensed_imu_yaw_rad}, {rpm_left}, {rpm_right}, {self.YAW}\n")


        if current_epoch_s - self.motor_start_time > 15.0:
            request_linear_x_vel = 0.0
            request_angular_z_vel = 0.0
            self.stop_robot = True


        else:
            request_linear_x_vel = Crequest_linear_x_vel
            request_angular_z_vel = Crequest_angular_z_vel

        twist_msg.linear.x = request_linear_x_vel
        twist_msg.angular.z = request_angular_z_vel
        self.cmd_vel_pub.publish(twist_msg)



def main():
    LaptopController()


if __name__ == "__main__":
    main()
