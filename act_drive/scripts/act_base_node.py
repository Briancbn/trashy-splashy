#!/usr/bin/env python2.7


from controller import Controller
from time import sleep
import rospy
from std_msgs.msg import Int16, Int32, Float32
from act_drive.msg import Act_drive
from math import isnan

if __name__ == '__main__':
    try:
        rospy.init_node('act_base')
        port = rospy.get_param('~port', '/dev/ttyACM0')
        baud = rospy.get_param('~baud', 115200)
        PubRate = rospy.Rate(20)
    
        ti = Controller(port, baud, "Tiva C Series TM4C123G")
        ti.connect()

        def drive_callback(msg):
            linear = msg.linear
            angular = msg.angular
            lift = msg.lift
            if isnan(linear):
                linear = 0
            if isnan(angular):
                angular = 0
            if isnan(lift):
                lift = 0
            ti.drive(linear, angular, lift);
            sleep(0.0001);

        encoder_pub = rospy.Publisher('act_base/encoder_FB', Int32, queue_size = 100)
        steer_pub = rospy.Publisher('act_base/steer_FB', Float32, queue_size = 100)
        left_light_pub = rospy.Publisher("act_base/left_light_control", Int16, queue_size = 100);
        right_light_pub = rospy.Publisher("act_base/right_light_control", Int16, queue_size = 100);
        mode_pub = rospy.Publisher("act_base/auto_mode", Int16, queue_size = 100);
        rospy.Subscriber("act_base/act_drive", Act_drive, drive_callback, queue_size = 1)

        encoder_base = (ti.get_encoder_counts())
        print("Start publishing message")

        while not rospy.is_shutdown():
            mode_pub.publish(int(ti.get_mode()))
            sleep(0.0001)
            encoder_pub.publish(int(ti.get_encoder_counts()) - encoder_base)
            sleep(0.0001)
            steer_pub.publish(float(ti.read_steer()) - 0.10)
            sleep(0.0001)
#            if light_state != None:
#                left_light_pub.publish(int(ti.get_light_state()[0]))
#                right_light_pub.publish(int(ti.get_light_state()[1]))
            PubRate.sleep()
    except rospy.ROSInterruptException:
        ti.close()
