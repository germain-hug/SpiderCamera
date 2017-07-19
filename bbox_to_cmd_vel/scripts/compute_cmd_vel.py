#!/usr/bin/env python
import rospy
import json
from collections import namedtuple
import design as d
from commands import update_motor_rel
from geometry_msgs.msg import Point
from bbox_to_cmd_vel.msg import cmd_vel_motors

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
<<<<<<< HEAD
        bbox_x, bbox_y = data[0], data[1]

=======
        bbox_x, bbox_y = data.x, data.y
>>>>>>> 65708bff52f0bb1cf09cc433ad5fa582edb5781d
    else:
        # Compute Displacement according to motion mode
    	msg = None
    	if(args[1].mode=='horizontal_topdown'):
            print("delta x:", data.x-bbox_x, "delta y:", data.y-bbox_y)
            msg = move_motor(data.x-bbox_x, data.y-bbox_y, 0.0)
        elif(args[1].mode=='vertical_side'):
            msg = move_motor(0.0, 0.0, data.x-bbox_x)

        # Update Coordinates and publish
        bbox_x, bbox_y = data.x, data.y
        if(msg is not None):
    		args[0].publish(msg)

# ---- Initialize ROS Node ----
def bbox_to_cmd_vel(run):
    rospy.init_node('bbox_to_cmd_vel', anonymous=True)
    pub = rospy.Publisher('cmd_vel', cmd_vel_motors, queue_size=1)
    rospy.Subscriber('bbox', Point, callback, (pub, run))
    rospy.spin()

# ---- Returns Relative Displacements and Updates Global Coordinates ----
def move_motor(x,y,z):
    # Compute new coordinates (cart. + spher.)
    delta = update_motor_rel(x, y, z, d.m)
    for i in range(0,len(d.m)):
        d.m[i] = [d.m[i][0] - x, d.m[i][1] - y, d.m[i][2] - z, d.m[i][3] + delta[i][0], d.m[i][4] + delta[i][1], d.m[i][5] + delta[i][2]]

    # Generate ROS Message
    msg = cmd_vel_motors()
    msg.vel_1 = delta[0][0]
    msg.vel_2 = delta[1][0]
    msg.vel_3 = delta[2][0]
    msg.vel_4 = delta[3][0]
    return msg

if __name__ == '__main__':
    d.__init__() # Compute initial Motors Positions

    with open('/home/hugo/catkin_ws/src/bbox_to_cmd_vel/scripts/run.json') as json_file:
        run = json.load(json_file)
    run = namedtuple('design', run.keys())(**run)

    bbox_to_cmd_vel(run)
