#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import time
# Initialize the ROS node
rospy.init_node('set_model_state')
def callback(data):
    position=data.pose[1]
    print(position)

# Publisher setup
state_pub = rospy.Subscriber('/steer_bot/gazebo/model_states', ModelStates, callback)

# Rate of publishing
rate = rospy.Rate(20)  # 20 Hz
start_time=time.time()
rospy.spin()