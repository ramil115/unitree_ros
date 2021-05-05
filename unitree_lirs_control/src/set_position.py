#!/usr/bin/env python

import rospy
from unitree_legged_msgs.msg import MotorCmd

def talker():
    pub = rospy.Publisher('/a1_gazebo/FR_hip_controller/command', MotorCmd, queue_size=10)
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = MotorCmd()
    msg.mode = 0x0A
    msg.Kp = 70;
    msg.dq = 0;
    msg.Kd = 3;
    msg.tau = 0;

    msg.q = 0;
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass