#!/usr/bin/env python
import rospy
import json
from collections import namedtuple
import design as d
from commands import update_motor_rel
from geometry_msgs.msg import Point
from bbox_to_cmd_vel.msg import cmd_vel_motors
from draw_splines import BezierBuilder

"""
-----------------------------------------
Takes Bounding Box coordinates as input,
Output 4 velocity commands for the motors
-----------------------------------------
"""

bbox_x = 0
bbox_y = 0
scaling_factor = 0.1

def callback(data, args):
    global bbox_x, bbox_y
    # Initialize first BB coordinates
    if bbox_x == 0 and bbox_y ==0:
        bbox_x, bbox_y = data.x, data.y
    else:
        # Compute Displacement according to motion mode
    	msg = None

        # ---- Horizontal (planar) Motion / Topdown view ----
    	if(args[1].mode=='horizontal_topdown' and not args[1].use_spline):
            if(abs((data.y-bbox_y)/100) > 0.3 or abs((data.x-bbox_x)/100) > 0.3):
                msg = move_motor((data.y-bbox_y)/10, (data.x-bbox_x)/10, 0.0)
            else:
                msg = move_motor(0.0, 0.0, 0.0)

        # ---- Vertical Motion / Side View ----
        elif(args[1].mode=='vertical_side' and not args[1].use_spline):
            if(abs((data.y-bbox_y)/100) > 0.4):
                msg = move_motor(0.0, 0.0, -(data.y-bbox_y)/10)
            else:
                msg = move_motor(0.0, 0.0, 0.0)

        # ---- Spline-based Motion Path ----
        elif(args[1].use_spline):
            # ------ TODO ------


        # Update Coordinates and publish
        #bbox_x, bbox_y = data.x, data.y
        if(msg is not None):
    		args[0].publish(msg)

# ---- Initialize ROS Node ----
def bbox_to_cmd_vel(run):
    rospy.init_node('bbox_to_cmd_vel', anonymous=True)
    pub = rospy.Publisher('cmd_vel', cmd_vel_motors, queue_size=1)
    rospy.Subscriber('bbox', Point, callback, (pub, run))
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
        spline_path = (bezier_builder.xp, bezier_builder.yp)

    # --- Start ROS Node ---
    bbox_to_cmd_vel(run)
