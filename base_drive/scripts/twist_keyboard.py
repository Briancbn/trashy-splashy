#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

import sys, select, termios, tty

# msg = """
# Reading from the keyboard  and Publishing to Twist!
# ---------------------------
# Moving around:
#    u    i    o
#    j    k    l
#    m    ,    .

# For Holonomic mode (strafing), hold down the shift key:
# ---------------------------
#    U    I    O
#    J    K    L
#    M    <    >

# t : up (+z)
# b : down (-z)

# anything else : stop

# q/z : increase/decrease max speeds by 10%
# w/x : increase/decrease only linear speed by 10%
# e/c : increase/decrease only angular speed by 10%

# CTRL-C to quit
# """

msg = """
Reading from the keyboard and Publishing to Twist!

   q    w    e
   a    s    d
   z    x    c

n/m : lift up/down

anything else : stop

l/k : increase/decrease max speeds by 10%
o/i : increase/decrease only linear speed by 10%
./, : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
        # 'i':(1,0,0,0),
        # 'o':(1,0,0,-1),
        # 'j':(0,0,0,1),
        # 'l':(0,0,0,-1),
        # 'u':(1,0,0,1),
        # ',':(-1,0,0,0),
        # '.':(-1,0,0,1),
        # 'm':(-1,0,0,-1),
        # 'O':(1,-1,0,0),
        # 'I':(1,0,0,0),
        # 'J':(0,1,0,0),
        # 'L':(0,-1,0,0),
        # 'U':(1,1,0,0),
        # '<':(-1,0,0,0),
        # '>':(-1,-1,0,0),
        # 'M':(-1,1,0,0),
        # 't':(0,0,1,0),
        # 'b':(0,0,-1,0),

        'q':(0,1,0,1),
        'w':(0,1,0,0),
        'e':(0,1,0,-1),
        'a':(0,0,0,1),
        'd':(0,0,0,-1),
        'z':(0,-1,0,-1),
        'x':(0,-1,0,0),
        'c':(0,-1,0,1),
}

liftBindings={
        'n':1,
        'm':-1,
}

speedBindings={
       'l':(1.1,1.1),
       'k':(.9,.9),
       'o':(1.1,1),
       'i':(.9,1),
       '.':(1,1.1),
       ',':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin,],[],[],0.0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('act_base/cmd_vel', Twist, queue_size = 1)
    pub_lift = rospy.Publisher('act_base/lift_vel', Int16, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')
    speed = rospy.get_param("~speed", 1.0)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    lift = 0
    status = 0
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                lift = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key in liftBindings.keys():
                x = 0
                y = 0
                z = 0
                th = 0
                lift = liftBindings[key]
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                lift = 0
                if (key == '\x03'):
                    break
            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)
            pub_lift.publish(lift)
            rospy.Rate(10).sleep()
    except:
        print e
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        lift = 0
        pub.publish(twist)
        pub_lift.publish(lift)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
