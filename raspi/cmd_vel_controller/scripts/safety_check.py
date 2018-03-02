#!/usr/bin/env python
import rospy
import json
from cmd_vel_controller.msg import cmd_vel_motors
from std_msgs.msg import Float32

"""
--------------------------------------------------
Performs a safety check on new velocity command,
transmits velocity to cmd_vel_controller node
--------------------------------------------------
"""

def callback(data, args):
	this_vel = 0
	dir = 1.0
	
	# Retrieve corresponding velocity value
	if(args[1]==1):
		this_vel = data.vel_1
	elif(args[1]==2):
		this_vel = data.vel_2
	elif(args[1]==3):
		this_vel = data.vel_3
	elif(args[1]==4):
		this_vel = data.vel_4
	
	# --- Publish velocity ---
	args[0].publish(this_vel) 

def cmd_vel_publisher(self_ID):
	
	# Define publishing topic according to Motor ID
	rospy.init_node('safety_check', anonymous=True)
	topic_out = 'cmd_vel_approved_' + str(self_ID);
	pub = rospy.Publisher(topic_out, Float32, queue_size=10)
	
	# Subscribe to the master commands
    	rospy.Subscriber('cmd_vel', cmd_vel_motors, callback, (pub, self_ID))
    	rospy.spin()

if __name__ == '__main__':
	with open('/home/pi/catkin_ws/specs.json') as data_file: 
		# Read Motor ID from JSON file
    		data = json.load(data_file)
		self_ID = data["ID"]
	cmd_vel_publisher(self_ID)
