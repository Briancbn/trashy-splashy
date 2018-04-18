#!/usr/bin/env python2.7


from controller import Controller
from time import sleep
import rospy
from std_msgs.msg import Bool, Int16, Int32, Float32
from base_drive.msg import Base_drive
from sensor_msgs.msg import Imu
from math import isnan

if __name__ == '__main__':
    try:
        rospy.init_node('base_drive')
        port = rospy.get_param('~port', '/dev/ttyACM0')
        baud = rospy.get_param('~baud', 115200)
        PubRate = rospy.Rate(20)

        teensy = Controller(port, baud, "Teensy3.2")
        teensy.connect()
        sleep(0.5)

        def drive_callback(msg):
            linear = msg.linear
            angular = msg.angular
            if isnan(linear):
                linear = 0
            if isnan(angular):
                angular = 0

            teensy.drive(linear, angular);
            sleep(0.0001);

        def e_stop_callback(msg):
            if msg.data:
                teensy.stop()
            else:
                teensy.start()
            sleep(0.0001)

        imu_state = Imu()
        imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        pid_pub = rospy.Publisher("pid", Float32, queue_size=1)
        rospy.Subscriber("base_drive", Base_drive, drive_callback, queue_size = 1)
        rospy.Subscriber("e_stop", Bool, e_stop_callback, queue_size = 1)
        print("Start publishing message")
        teensy.stop()

        while not rospy.is_shutdown():
            teensy.get_imu_state()
            angular_speed = Float32()
            angular_speed.data = teensy.angular_speed
            pid_pub.publish(angular_speed)
            imu_pub.publish(teensy.imuMsg)
            sleep(0.001)
            PubRate.sleep()

    except KeyboardInterrupt:
        teensy.stop()
        sleep(0.5)
        teensy.close()
