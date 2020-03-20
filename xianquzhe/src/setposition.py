#! /usr/bin/env python
import rospy
import math
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float64
import sys, select, termios, tty



def SetJointState():
    #Initializes the publisher and message
    pub0 = rospy.Publisher('xianquzhe/up_position_controller/command', Float64, queue_size=10)
    pub1 = rospy.Publisher('xianquzhe/ur1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('xianquzhe/ur2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('xianquzhe/ur3_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('xianquzhe/ur4_position_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('xianquzhe/ur5_position_controller/command', Float64, queue_size=10)
    pub6 = rospy.Publisher('xianquzhe/ur6_position_controller/command', Float64, queue_size=10)
    up_msg = Float64()
    ur1_msg = Float64()
    ur2_msg = Float64()
    ur3_msg = Float64()
    ur4_msg = Float64()
    ur5_msg = Float64()
    ur6_msg = Float64()

    rate = rospy.Rate(1)
    rate.sleep()


    #change and send the message
    up_msg = 0.2
    ur1_msg.data = -math.pi/2
    ur2_msg.data = 0
    ur3_msg.data = -0.5
    ur4_msg.data = 0.5
    ur5_msg.data = 0
    ur6_msg.data = math.pi/2
    pub0.publish(up_msg)
    pub1.publish(ur1_msg)
    pub2.publish(ur2_msg)
    pub3.publish(ur3_msg)
    pub4.publish(ur4_msg)
    pub5.publish(ur5_msg)
    pub6.publish(ur6_msg)




if __name__ == '__main__':
    rospy.init_node('ur_state_publisher')
    try:
        rate = rospy.Rate(1)
        rate.sleep()
        SetJointState()

    except rospy.ROSInterruptException:
        pass