#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import numpy as np
import six.moves.urllib as urllib
from std_msgs.msg import Float32
import sys
import os
import math


class deep_navigation:
    
    def __init__(self):
        self.steer_sub = rospy.Subscriber("steer", Float32, self.steer_update)
        self.coll_sub = rospy.Subscriber("coll", Float32, self.coll_update)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.alpha = 0.5
        self.beta = 0.7
        # original collision probability and steering direction
        self.coll = 1
        self.steer = 0
        # original position
        self.time_interval = 4
        self.loc_x = 0
        self.loc_y = 0
        self.curr_angle = 0

    def steer_update(self, data):
        self.steer = data.data

    def coll_update(self, data):
        self.coll = data.data
        self.move()

    def move(self, adjust_interval = 3):
        vel_msg = Twist()
        t0 = rospy.Time.now().to_sec()

        if self.time_interval >= adjust_interval:
            self.time_interval = 0
            vel_msg.linear.x = 0

        # move to avoid collision       
        if self.coll <= 0.9:
            vel_msg.linear.x = 0.05
            print("forward")
        else:
            vel_msg.linear.x = 0
            print("stop")
        
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # vel_msg.angular.z = self.steer / 1.58
        if self.steer == 0:
            vel_msg.angular.z = 0
        elif self.steer < 0:
            vel_msg.angular.z = -1
        else:
            vel_msg.angular.z = 1       
        self.velocity_publisher.publish(vel_msg)     
        t1=rospy.Time.now().to_sec()
       
        # calculate current position
        self.curr_angle += vel_msg.angular.z * 2 * np.pi * (t1 - t0)
        forward_dis =  vel_msg.linear.x * (t1 - t0)
        self.loc_x += forward_dis * np.cos(np.float(self.curr_angle))
        self.loc_y += forward_dis * np.sin(np.float(self.curr_angle))

        # adjust direction
        self.time_interval += t1 - t0
        



        

        # adust towards destination


  
    
def main(args):
    rospy.init_node('deep_navigation', anonymous=True)
    dn = deep_navigation()
    try:
        # dn.move()
        rospy.spin()
    except KeyboardInterrupt:
        print("STOP TRIP")



if __name__ == '__main__':
        main(sys.argv)