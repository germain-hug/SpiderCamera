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
  	pub.publish(compute_cmd_vel(data))

def bbox_to_cmd_vel():
    rospy.init_node('bbox_to_cmd_vel', anonymous=True)
    pub = rospy.Publisher('cmd_vel', cmd_vel_motors, queue_size=1)
    rospy.Subscriber('bbox', Point, callback, pub)
    rospy.spin()

def compute_cmd_vel(data):

    print(update_motor_rel(0.0, 0.0, 0.0, (d.m_1, d.m_2, d.m_3, d.m_4)))

    # Generate Motors Command
    msg = cmd_vel_motors()
    msg.vel_1 = 0.1
    msg.vel_2 = 0.1
    msg.vel_3 = 0.1
    msg.vel_4 = 0.1
    return msg

if __name__ == '__main__':
    d.__init__() # Compute initial Motors Positions
    bbox_to_cmd_vel()
