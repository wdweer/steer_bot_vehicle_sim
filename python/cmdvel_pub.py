#! /usr/bin/env python

"""
Twist Publisher example

referenced from wiki.ros.org

url: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
"""

import time
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("drive_forward")
pub = rospy.Publisher("/steer_bot/ackermann_steering_controller/cmd_vel", Twist, queue_size=1)
r = rospy.Rate(1000)  # 1 Hz

forward = Twist()
stop = Twist()

forward.linear.x = 0.5

stop.linear.x = 0.0

start_time = time.time()

rospy.loginfo("==== DriveForward node Started, move forward during 5 seconds ====\n")


while not rospy.is_shutdown():
    if time.time() - start_time < 20.0:
        pub.publish(forward)
    else:
        rospy.logwarn(" 20 seconds left, Stop!! ")
        pub.publish(stop)
        break
    r.sleep()