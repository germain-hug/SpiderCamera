#!/usr/bin/env python
import rospy
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

def callback(data, pub):
    msg = move_motor(0.0,0.0,0.0)
    pub.publish(msg)

# ---- Initialize ROS Node ----
def bbox_to_cmd_vel():
    rospy.init_node('bbox_to_cmd_vel', anonymous=True)
    pub = rospy.Publisher('cmd_vel', cmd_vel_motors, queue_size=1)
    rospy.Subscriber('bbox', Point, callback, pub)
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
    bbox_to_cmd_vel()
