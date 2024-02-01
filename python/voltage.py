#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import time
rospy.init_node('voltage_publisher')

volt_pub1=rospy.Publisher('/Voltage_sub', Float32, queue_size=1)


x=Float32()
x.data=10
r=rospy.Rate(10)
y=Float32()
y.data=0
start_time=time.time()

while not rospy.is_shutdown():
    if time.time()-start_time < 10:
       volt_pub1.publish(x)
      
    else:
       volt_pub1.publish(y)
      
    r.sleep()