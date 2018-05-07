#!/usr/bin/env python2.7

import thread
from serial import Serial
from serial.serialutil import SerialException
import sys, traceback, os
import time
from command import cmdDict
from time import sleep

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

class Controller(object):
    def __init__(self, port = "/dev/ttyACM0", baud = 115200, device = "Teensy3.2", timeout = 0.5):
        self.port = port
        self.baud = baud
        self.device = device
        self.timeout = timeout
        self.writeTimeout = timeout
        self.encoder_counts = 0
        self.steering = 0
        self.radar_state = 0
        self.mutex = thread.allocate_lock()

        self.imuMsg = Imu()
        self.seq = 0
        self.last_yaw = 0
        self.angular_speed = 0

        # Orientation covariance estimation:
        # Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
        # Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
        # Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
        # cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
        # i.e. variance in yaw: 0.0025
        # Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
        # static roll/pitch error of 0.8%, owing to gravity orientation sensing
        # error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
        # so set all covariances the same.
        self.imuMsg.orientation_covariance = [
        0.0025 , 0 , 0,
        0, 0.0025, 0,
        0, 0, 0.0025
        ]

        # Angular velocity covariance estimation:
        # Observed gyro noise: 4 counts => 0.28 degrees/sec
        # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
        # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
        self.imuMsg.angular_velocity_covariance = [
        0.02, 0 , 0,
        0 , 0.02, 0,
        0 , 0 , 0.02
        ]

        # linear acceleration covariance estimation:
        # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
        # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
        # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
        self.imuMsg.linear_acceleration_covariance = [
        0.04 , 0 , 0,
        0 , 0.04, 0,
        0 , 0 , 0.04
        ]

        self.accel_factor = 9.806 / 256.0


    def connect(self):
        try:
            print "Connecting to controller on port", self.port, "..."
            self.ser = Serial(port = self.port, baudrate = self.baud, timeout = self.timeout, writeTimeout = self.writeTimeout)
            sleep(1)
            test = self.get_baud()
            if test != self.baud:
                attempt = 0
                while(True):
                    attempt += 1
                    print "Retrying..."
                    if attempt > 5:
                        raise SerialException
                    sleep(1)
                    test = self.get_baud()
                    print test
                    if test == self.baud:
                        break
            print "Connected at", self.baud
            sleep(0.5)
            test = self.get_device()
            if test != self.device:
                print "Unknown device for base controller:", test
                raise SerialException
            print "Connected with", self.device

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Connect Fail"
            os._exit(1)


    def send(self, cmd):
        self.ser.write(cmd + '\r')

    def close(self):
        self.ser.close()

    def recv_raw(self):
        msg = ""
        value = ""
        attempts = 0
        while msg != '\r':
            msg = self.ser.read(1)
            value += msg
            attempts += 1
            if attempts > 30:
                return None
        value = value.strip("\r")
        value = value.strip("\n")
        return value


    def recv_ack(self):
        return "OK"

    def recv_int(self):
        value = self.recv_raw()
        try:
            return int(value)
        except:
            return None

    def recv_float(self):
        value = self.recv_raw()
        try:
            return float(value)
        except:
            return None


    def recv(self, mode = "RAW"):
        if mode == "INT":
            return self.recv_int()
        elif mode == "FLOAT":
            return self.recv_float()
        elif mode == "ACK":
            return self.recv_raw() == "OK"
        elif mode == "RAW":
            return self.recv_raw()
        else:
            return None


    def execute(self, cmd, mode = "RAW"):
        self.mutex.acquire()
        tries = 1
        attempts = 0
        self.send(cmd)
        try:
            recvmsg = self.recv(mode)
            while(attempts < tries and (recvmsg == "" or recvmsg == None)):
                try:
                    self.ser.flushInput()
                    self.send(cmd)
                    recvmsg = self.recv(mode)
                except:
                    print "Exception executing command: " + cmd
                    attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            self.stop()
            recvmsg = None
        self.mutex.release()
        return recvmsg


    def deg_to_rad(self, deg):
        return 3.1415926 / 180.0 * deg

    def rad_to_deg(self, rad):
        return 180 / 3.1415926 * rad


    def steer_to_deg(self, steer):
        return steer * (-0.080)

    def get_baud(self):
        try:
            return self.execute(cmdDict["GET_BAUDRATE"], "INT")
        except:
            return None

    def get_device(self):
        try:
            return self.execute(cmdDict["GET_DEVICE"], "RAW")
        except:
            return None

    def start(self):
        try:
            return self.execute(cmdDict["START"], "ACK")
        except:
            return None

    def stop(self):
        try:
            return self.execute(cmdDict["STOP"], "ACK")
        except:
            return None

    def get_imu_state(self):
        try:
            yaw = self.execute(cmdDict["GET_YAW"], "FLOAT")
            self.angular_speed = -20 * (yaw - self.last_yaw)
            if self.angular_speed > 300:
                self.angular_speed = 0
            if self.angular_speed < -300:
                self.angular_speed = 0
            self.last_yaw = yaw
            pitch = self.execute(cmdDict["GET_PITCH"], "FLOAT")
            roll = self.execute(cmdDict["GET_ROLL"], "FLOAT")
            ax = self.execute(cmdDict["GET_AX"], "FLOAT")
            ay = self.execute(cmdDict["GET_AY"], "FLOAT")
            az = self.execute(cmdDict["GET_AZ"], "FLOAT")
            gx = self.execute(cmdDict["GET_GX"], "FLOAT")
            gy = self.execute(cmdDict["GET_GY"], "FLOAT")
            gz = self.execute(cmdDict["GET_GZ"], "FLOAT")


            #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
            if yaw > 180.0:
                yaw = yaw - 360.0
            if yaw <= -180.0:
                yaw = yaw + 360.0
            yaw_rad = self.deg_to_rad(yaw)
            #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
            pitch_rad = -self.deg_to_rad(pitch)
            roll_rad = self.deg_to_rad(roll)

            # Publish message
            # AHRS firmware accelerations are negated
            # This means y and z are correct for ROS, but x needs reversing
            self.imuMsg.linear_acceleration.x = -ax * self.accel_factor
            self.imuMsg.linear_acceleration.y = ay * self.accel_factor
            self.imuMsg.linear_acceleration.z = az * self.accel_factor

            self.imuMsg.angular_velocity.x = gx
            #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
            self.imuMsg.angular_velocity.y = -gy
            #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
            self.imuMsg.angular_velocity.z = -gz

            q = quaternion_from_euler(roll_rad,pitch_rad,yaw_rad)
            self.imuMsg.orientation.x = q[0]
            self.imuMsg.orientation.y = q[1]
            self.imuMsg.orientation.z = q[2]
            self.imuMsg.orientation.w = q[3]
            self.imuMsg.header.stamp= rospy.Time.now()
            self.imuMsg.header.frame_id = 'base_imu_link'
            self.imuMsg.header.seq = seq
            seq = seq + 1
        except:
            return None

    def drive(self, linear, angular):
            cmd = cmdDict["LINEAR_INPUT"] + " " + str(int(linear))
            self.execute(cmd, "ACK")
            cmd = cmdDict["ANGULAR_INPUT"] + " " + str(round(angular, 2))
            self.execute(cmd, "ACK")
