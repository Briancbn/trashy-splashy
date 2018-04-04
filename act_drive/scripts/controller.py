#!/usr/bin/env python2.7

import thread
from serial import Serial
from serial.serialutil import SerialException
import sys, traceback, os
import time
from command import cmdDict
from time import sleep

class Controller(object):
    def __init__(self, port = "/dev/ttyACM0", baud = 115200, device = "Tiva C Series TM4C123G", timeout = 0.5):
        self.port = port
        self.baud = baud
        self.device = device
        self.timeout = timeout
        self.writeTimeout = timeout
        self.encoder_counts = 0
        self.steering = 0
        self.radar_state = 0
        self.mutex = thread.allocate_lock()


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

        return value


    def recv_ack(self):
        return "OK"

    def recv_int(self):
        value = self.recv_raw()
        try:
            return int(value, 0)
        except:
            return None

    def recv(self, mode = "RAW"):
        if mode == "INT":
            return self.recv_int()
        elif mode == "ACK":
            return self.recv_raw()
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
            recvmsg = None
        self.mutex.release()
        return recvmsg

    
    def deg_to_rad(self, deg):
        return 3.1415926 / 180.0 * deg

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

    def get_encoder_counts(self):
        self.encoder_counts = self.execute(cmdDict["READ_ENCODERS"], "INT")
        return self.encoder_counts

    def read_steer(self):
        self.steering = self.deg_to_rad(self.steer_to_deg(self.execute(cmdDict["READ_STEER"], "INT") + 70))
        return self.steering


    def reset_encoder(self):
        self.encoder_counts = 0
        return self.execute(cmdDict["RESET_ENCODERS"], "ACK")


    def get_radar_state(self):
        self.radar_state = self.execute(cmdDict["READ_RADAR"], "INT")
        try:
            self.radar_state = int(self.radar_state)
        except:
            self.radar_state = 0
        return self.radar_state

    def change_light_state(self, light_type, state):
        print state;
        cmd = cmdDict[light_type + "_SIG"] + " " + hex(state);
        self.execute(cmd, "ACK")

    def get_light_state(self):
        light_state = self.execute(cmdDict["GET_LIGHT_STATE"], "INT");
        if light_state == 0:
            return (0,0)
        elif light_state == 1:
            return (2,0)
        elif light_state == -1:
            return (0,2)
        else:
            return None

    def get_mode(self):
        mode = self.execute(cmdDict["GET_MODE"], "INT")
        return mode

    def drive(self, linear, angular, lift):
        if lift == 0:
            if linear < 0.3 and linear > 0:
                linear = 0
            elif linear > -0.3 and linear <0:
                linear = 0
            if angular < 0.05 and angular > 0:
                angular = 0
            elif angular > -0.05 and angular <0:
                angular = 0
            cmd = cmdDict["DRIVE"] + " " + hex(int(-linear * 1000))
            self.execute(cmd, "ACK")
            cmd = cmdDict["STEER"] + " " + hex(int(angular * 1000))
            self.execute(cmd, "ACK")
            cmd = cmdDict["LIFT"] + " " + hex(0)
            self.execute(cmd, "ACK")      
        else:
            cmd = cmdDict["DRIVE"] + " " + hex(int(0))
            self.execute(cmd, "ACK")
            cmd = cmdDict["STEER"] + " " + hex(int(0))
            self.execute(cmd, "ACK")
            cmd = cmdDict["LIFT"] + " " + hex(int(lift))
            self.execute(cmd, "ACK")      
