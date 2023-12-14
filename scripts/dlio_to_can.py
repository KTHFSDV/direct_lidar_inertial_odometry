#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
# from std_msgs.msg import Float64
from ros2can_msgs.msg import SBG_ECAN_MSG_ODO_VEL, SBG_ECAN_MSG_EKF_EULER
import math 

###
# SBG_ECAN_MSG_EKF_EULER:
# 306
# int16  ROLL 
# int16  PITCH
# int16  YAW  
# SBG_ECAN_MSG_ODO_VEL:
# 353
# int16  VELOCITY


def main():
    rospy.init_node('dlio_2_can')

    # Publishers
    sbg_euler_pub = rospy.Publisher('/ros2can/send/SBG_ECAN_MSG_EKF_EULER', SBG_ECAN_MSG_EKF_EULER, queue_size=10)
    sbg_odo_vel_pub = rospy.Publisher('/ros2can/send/SBG_ECAN_MSG_ODO_VEL', SBG_ECAN_MSG_ODO_VEL, queue_size=10)
    
    rospy.loginfo("Node initialized, now waiting for Odometry messages...")

    # Wait for  for publishers to set up
    rospy.sleep(1.0)

    # Subscribe to Odometry messages and pass the publishers as arguments
    rospy.Subscriber('/robot/dlio/odom_node/odom', Odometry, odom_callback, (sbg_euler_pub, sbg_odo_vel_pub))

    rospy.spin()

def odom_callback(odom_msg, args):
    # Unpack the arguments
    sbg_euler_pub, sbg_odo_vel_pub = args  

    sbg_msg_ekf = SBG_ECAN_MSG_EKF_EULER()
    sbg_msg_odo = SBG_ECAN_MSG_ODO_VEL()

    # Access orientation components  (x, y, z, w) 
    sbg_msg_ekf.ROLL = odom_msg.pose.pose.orientation.x
    sbg_msg_ekf.PITCH = odom_msg.pose.pose.orientation.y
    sbg_msg_ekf.YAW = odom_msg.pose.pose.orientation.z

    # Assign linear velocity to  message field
    sbg_msg_odo.VELOCITY = odom_msg.twist.twist.linear.x

    # If the velocity should be like sqrt(x^2+y^2)
    
    # vel_x = odom_msg.twist.twist.linear.x
    # vel_y = odom_msg.twist.twist.linear.y
    # vel_tot = math.sqrt(vel_x**2 + vel_y**2)
    # sbg_msg_odo.VELOCITY = vel_tot
    
    rospy.loginfo("Published SBG messages")

    # Publish messages
    sbg_euler_pub.publish(sbg_msg_ekf)
    sbg_odo_vel_pub.publish(sbg_msg_odo)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
