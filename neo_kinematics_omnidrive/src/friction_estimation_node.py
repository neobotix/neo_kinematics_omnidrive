#!/usr/bin/env python
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plot
import matplotlib.animation as animation
import random
import rospy
from std_msgs.msg import Float64
import time

def main():
	# Generating parameters
	time_set = np.arange(0., 60.01, 0.01)
	velocity_set = np.arange(-3.0,3.0001,0.001)
	# velocity_data = np.reshape(velocity_set,(len(velocity_set),1))
	# ROS Node
	rospy.init_node("friction_estimation_node")
	rospy.loginfo("friction_node started")
	velocity = rospy.Publisher('velocity', Float64, queue_size=10)
	loop_rate = rospy.Rate(100)
	vel = Float64() #Creating a velocity message
	now = int(round(time.time() * 1000))
	while not rospy.is_shutdown():
		time_count = int(round(time.time() * 1000)) - now
		for i in range(len(time_set)): 
			if(time_count==time_set[i]*1000):
				vel.data = velocity_set[i]
				velocity.publish(vel)
		loop_rate.sleep()


if __name__ == '__main__':
	main()
