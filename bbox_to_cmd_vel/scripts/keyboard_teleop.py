#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import design as d
import numpy as np
import sys, select, termios, tty
from bbox_to_cmd_vel.msg import cmd_vel_motors
from compute_cmd_vel import move_motor
from commands import update_motor_rel
import subprocess as sp

splash = """
---------------------------
 Reading from the keyboard
---------------------------

 Horizontal Translation :

       'w'
        ▲
  'a' ◀   ▶ 'd'
	▼
       's'

 Vertical Translation :

 'u'/'j' : Up ▲ / Down ▼

 Space Bar to stop
 'q' to quit
"""

# ===================================

moveBindings = {
		'w':(1.0,0.0,0.0), # Forward
		's':(-1.0,0.0,0.0),# Backward
		'a':(0.0,1.0,0.0), # Left
		'd':(0.0,-1.0,0.0),# Right
		'u':(0.0,0.0,1.0), # Up
		'j':(0.0,0.0,-1.0),# Down
		}


DAMP_DURATION = 1.0 # in seconds

# ===================================


def getKey():
	if(select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])):
		return sys.stdin.read(1)
	return False

	"""
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key
	"""

if __name__=="__main__":

	settings = termios.tcgetattr(sys.stdin)
	tty.setcbreak(sys.stdin.fileno())

	rospy.init_node('keyboard_teleop', anonymous=True)
	pub = rospy.Publisher('cmd_vel', cmd_vel_motors, queue_size=1)

	curr_vel = (0.0,0.0,0.0)
	new_vel = (0.0,0.0,0.0)
	old_delta=(0.0, 0.0, 0.0, 0.0)

	d.__init__()

	try:
		sp.call('clear',shell=True)
		print splash

		t_start = time.time()
		start = False

		while(1):
			key = getKey()

			# ---- A new command has been emitted ---
			if key in moveBindings.keys():
				new_vel = np.asarray(np.add(new_vel,moveBindings[key]))
				t_start = time.time()
				if 'msg' in locals():
					old_delta= (msg.vel_1, msg.vel_2, msg.vel_3, msg.vel_4)
				start = True

			# ---- A 'stop' command has been emitted ---
			elif(key==' '):
				t_start = time.time()
				new_vel = (0.0, 0.0, 0.0)
				start = True

			elif(key=='q'):
				break

			# ----- linerp factor ----
			t_linerp = min((time.time() - t_start) / DAMP_DURATION, 1.0)
			if(t_linerp == 1.0):
				start = False
				old_delta= (msg.vel_1, msg.vel_2, msg.vel_3, msg.vel_4)

			elif not start:
				t_start = time.time()


			# ---- Speed interpolation ---
			if start:
				msg = move_motor(new_vel[0], new_vel[1], new_vel[2], t_linerp, old_delta)
				pub.publish(msg)

	except:
		print " ---- Error ----"

	finally:
		print(" ---- Stopping all motors ----")
		msg = move_motor(0.0, 0.0, 0.0, 1.0, (0.0, 0.0, 0.0, 0.0))
		pub.publish(msg)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
