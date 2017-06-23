#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from bbox_to_cmd_vel.msg import cmd_vel_motors

"""
-----------------------------------------
Takes Bounding Box coordinates as input,
Output 4 velocity commands for the motors
-----------------------------------------
"""

def callback(data, pub):
  	pub.publish(compute_cmd_vel(data))

def bbox_to_cmd_vel():
    rospy.init_node('bbox_to_cmd_vel', anonymous=True)
    pub = rospy.Publisher('cmv_vel', cmd_vel_motors, queue_size=1)
    rospy.Subscriber('bbox', Point, callback, pub)
    rospy.spin()

def compute_cmd_vel(data):

    # Generate Motors Command
    msg = cmd_vel_motors()
    msg.vel_1 = 0.1
    msg.vel_2 = 0.1
    msg.vel_3 = 0.1
    msg.vel_4 = 0.1
    return msg

if __name__ == '__main__':
    bbox_to_cmd_vel()
