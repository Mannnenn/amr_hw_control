#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('cmd_vel_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

twist = Twist()
twist.linear.x = 1.0  # set linear speed to 0.1 m/s
twist.angular.z = 0  # set angular speed to 0.5 rad/s
pub.publish(twist)
rospy.sleep(1)

twist.linear.x = 0.0  # set linear speed to 0.1 m/s
twist.angular.z = 0  # set angular speed to 0.5 rad/s
pub.publish(twist)
