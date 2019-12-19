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
	time_set = np.arange(0., 100.01, 0.01)
	freq = 1
	np.random.seed(10)
	amplitude_a_offset = np.sin(time_set*freq)+1.2
	# ROS Node
	rospy.init_node("excitation_node")
	rospy.loginfo("Excitation_node started")
	velocity = rospy.Publisher('velocity', Float64, queue_size=10)
	loop_rate = rospy.Rate(100)
	vel = Float64() #Creating a velocity message
	now = int(round(time.time() * 1000))
	with open('excitation_steer.txt', 'w') as f:
  	# print >>f, number
		while not rospy.is_shutdown():
			time_count = int(round(time.time() * 1000)) - now
			for i in range(len(time_set)): 
				if(time_count==time_set[i]*1000):
					vel.data = amplitude_a_offset[i]
					velocity.publish(vel)
					print >>f, rospy.get_rostime()
			loop_rate.sleep()


if __name__ == '__main__':
	main()
