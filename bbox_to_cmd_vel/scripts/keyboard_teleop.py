#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import design as d
import sys, select, termios, tty
from bbox_to_cmd_vel.msg import cmd_vel_motors
from compute_cmd_vel import move_motor

SF = 15 # Speed Factor

splash = """
--------------------------
Reading from the keyboard
--------------------------

Horizontal Translation :

       'z'    
        ▲
  'q' ◀   ▶ 'd'
	▼
       's'    

Vertical Translation :

'u'/'j' : Up ▲ / Down ▼

Any other key to stop
CTRL-C to quit
"""


moveBindings = {
		'z':(1.0,0.0,0.0), # Forward
		's':(-1.0,0.0,0.0),# Backward
		'q':(0.0,1.0,0.0), # Left
		'd':(0.0,-1.0,0.0),# Right
		'u':(0.0,0.0,1.0), # Up
		'j':(0.0,0.0,-1.0),# Down
		}


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":

	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('keyboard_teleop', anonymous=True)
	pub = rospy.Publisher('cmd_vel', cmd_vel_motors, queue_size=1)
	
	curr_vel = (0,0,0)
	d.__init__()

	try:
		print splash
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				curr_vel = (curr_vel[0]+x, curr_vel[1]+y, curr_vel[2]+z)
			else:
				curr_vel = (0.0,0.0,0.0)
				if (key == '\x03'):
					break

			msg = move_motor(SF*curr_vel[0],SF*curr_vel[1],SF*curr_vel[2])
			#print(d.m)
			pub.publish(msg)

	except:
		print e

	finally:
		print(" ---- Stopping all motors ----")
		msg = move_motor(0.0,0.0,0.0)
		pub.publish(msg)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

