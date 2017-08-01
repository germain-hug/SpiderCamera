#!/usr/bin/env python
import rospy
import json
from collections import namedtuple
import design as d
from commands import update_motor_rel
from geometry_msgs.msg import Point
from bbox_to_cmd_vel.msg import cmd_vel_motors

from draw_splines import BezierBuilder
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import threading
from Queue import Queue
import time

"""
-----------------------------------------
Takes Bounding Box coordinates as input,
Output 4 velocity commands for the motors
-----------------------------------------
"""

# Bounding Box Coordinates
bbox_x = 0
bbox_y = 0
# Velocity Path (if Spline-based)
x_v = []
y_v = []
# Spline Current index
idx = 0


# ***************************
# ---- Publishing Thread ----
# ***************************

def command_thread(q, pub):

    # Initial Values
    t_start = time.time()
    start = False
    curr_vel = (0.0,0.0,0.0)
	new_vel = (0.0,0.0,0.0)
	old_delta=(0.0, 0.0, 0.0, 0.0)

    while True:

        # ----- New Velocity Command has been published ----
        cmd = q.get()
        if cmd is not None:
            new_vel = np.asarray(np.add(new_vel,moveBindings[key]))
            t_start = time.time()
            if 'msg' in locals():
                old_delta= (msg.vel_1, msg.vel_2, msg.vel_3, msg.vel_4)
            start = True

        # ----- Linear Interpolation Factor ----
        t_linerp = min((time.time() - t_start) / DAMP_DURATION, 1.0)
        if(t_linerp == 1.0):
            start = False
            old_delta= (msg.vel_1, msg.vel_2, msg.vel_3, msg.vel_4)

        elif not start:
            t_start = time.time()

        # ----- Finally, publish ----
        if start:
            msg = move_motor(FACTOR*new_vel[0], FACTOR*new_vel[1], FACTOR*new_vel[2], t_linerp, old_delta)
            pub.publish(msg)


# ***************************
# ---- Callback Function ----
# ***************************

def callback(data, args):
    global bbox_x, bbox_y, x_v, y_v, idx
    q = args[2]
    # Initialize first BB coordinates
    if bbox_x == 0 and bbox_y ==0:
        bbox_x, bbox_y = data.x, data.y
    else:

        # ===================================================
        # ---- Horizontal (planar) Motion / Topdown view ----
        # ===================================================

    	if(args[1].mode=='horizontal_topdown' and not args[1].use_spline):
            if(abs((data.y-bbox_y)/100) > 0.3 or abs((data.x-bbox_x)/100) > 0.3):
                q.put((data.y-bbox_y)/10, (data.x-bbox_x)/10, 0.0)
            else:
                q.put(0.0, 0.0, 0.0)

        # =====================================
        # ---- Vertical Motion / Side View ----
        # =====================================

        elif(args[1].mode=='vertical_side' and not args[1].use_spline):
            if(abs((data.y-bbox_y)/100) > 0.3):
                q.put(0.0, 0.0, -(data.y-bbox_y)/10)
            else:
                q.put(0.0, 0.0, 0.0)

        # ==================================
        # ---- Spline-based Motion Path ----
        # ==================================

        elif(args[1].use_spline):
            if(args[1].plane=='xy'):
                if((data.y-bbox_y)/100 > 0.3): # Forward on spline
                    idx = min(idx+1, len(x_v))
                    q.put(y_v[idx]*100, x_v[idx]*10, 0.0)
                elif((data.y-bbox_y)/100 < 0.3): # Backward on spline
                    idx = max(0, idx-1)
                    q.put(y_v[idx]*100, x_v[idx]*10, 0.0)
                else:
                    q.put(0.0, 0.0, 0.0)


# --- Start Publishing Thread and ROS Node ---
def bbox_to_cmd_vel(run, queue):
    rospy.init_node('bbox_to_cmd_vel', anonymous=True)
    pub = rospy.Publisher('cmd_vel', cmd_vel_motors, queue_size=1)

    queue = Queue(maxsize = 1)
    thread = threading.Thread(target=command_thread, args=(queue, pub))
    thread.daemon = True
    thread.start()

    rospy.Subscriber('bbox', Point, callback, (pub, run, queue))
    rospy.spin()

# ---- Returns Relative Displacements and Updates Global Coordinates ----
def move_motor(x,y,z,f=1.0,old_delta=(0.0,0.0,0.0,0.0)):
    # Compute new coordinates (cart. + spher.)
    delta = update_motor_rel(x, y, z, d.m)
    for i in range(0,len(d.m)):
        #d.m[i] = [d.m[i][0] - x, d.m[i][1] - y, d.m[i][2] - z, d.m[i][3] + delta[i][0], d.m[i][4] + delta[i][1], d.m[i][5] + delta[i][2]]
        d.m[i] = [d.m[i][0], d.m[i][1], d.m[i][2], d.m[i][3] + delta[i][0], d.m[i][4] + delta[i][1], d.m[i][5] + delta[i][2]]

    # Generate ROS Message
    msg = cmd_vel_motors()
    msg.vel_1 = round(delta[0][0]*f + old_delta[0]*(1-f), 3)
    msg.vel_2 = round(-delta[1][0]*f + old_delta[1]*(1-f), 3)
    msg.vel_3 = round(delta[2][0]*f + old_delta[2]*(1-f), 3)
    msg.vel_4 = round(-delta[3][0]*f + old_delta[3]*(1-f), 3)
    return msg

if __name__ == '__main__':

    # --- Initial Motors Coordinates ---
    d.__init__()

    # --- Hyperparams ---
    with open('/home/hugogermain/catkin_ws/src/bbox_to_cmd_vel/scripts/run.json') as json_file:
        run = json.load(json_file)
    run = namedtuple('design', run.keys())(**run)

    # --- Retrieve User Motion Path (Optional) ---
    if run.use_spline:
        global x_v, y_v
        fig, ax1 = plt.subplots(1, 1, figsize=(5, 5))
        line = Line2D([], [], ls='--', c='#666666',
                    marker='x', mew=2, mec='#204a87')

        ax1.add_line(line)
        ax1.set_xlim(-1, 1)
        ax1.set_ylim(-1, 1)
        ax1.set_title("Bezier curve")

        # Create BezierBuilder
        bezier_builder = BezierBuilder(line)
        plt.show()

        x_s, y_s = bezier_builder._build_bezier()

        # Compute Velocity Spline Commands
        first_order = lambda s, i: s[i+1]-s[i]
        x_v = [first_order(x_s, i) for i in range(0,len(x_s)-1)]
        y_v = [first_order(y_s, i) for i in range(0,len(y_s)-1)]

    bbox_to_cmd_vel(run, queue)
