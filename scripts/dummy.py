#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

# Define global variables
r_vel = 0.0
l_vel = 0.0


class DummyNode:
    def __init__(self):
        # Subscribe to "motor_command" topic and pass callback function
        self.sub = rospy.Subscriber(
            "motor_command", Float32MultiArray, self.callback)
        self.rate = rospy.Rate(10)  # 10Hz

    def callback(self, msg):
        # Update global variables with received message
        global r_vel
        global l_vel
        r_vel = msg.data[0]
        l_vel = msg.data[1]


if __name__ == '__main__':
    rospy.init_node('dummy_node')
    node = DummyNode()
    pub = rospy.Publisher("motor_response", Float32MultiArray, queue_size=1)
    while not rospy.is_shutdown():

        response_msg = Float32MultiArray()
        response_msg.data = [r_vel, l_vel]
        pub.publish(response_msg)

        rospy.sleep(0.05)
