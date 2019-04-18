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
    def __init__(self,dest_x = 5. , dest_y = 5. ):
        self.steer_sub = rospy.Subscriber("steer", Float32, self.steer_update)
        self.coll_sub = rospy.Subscriber("coll", Float32, self.coll_update)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.alpha = 0.5
        self.beta = 0.7
        # original collision probability and steering direction
        self.coll = 1
        self.steer = 0
        # original position
        self.time_interval = 5.
        self.loc_x = 0.
        self.loc_y = 0.
        self.curr_angle = 0.
        self.dest_x = dest_x
        self.dest_y = dest_y

    def steer_update(self, data):
        self.steer = data.data

    def coll_update(self, data):
        self.coll = data.data
        self.move()

    def move(self, adjust_interval = 5.):
        sgn = lambda x: 1. if x > 0 else -1. if x < 0 else 0.
        
        vel_msg = Twist()
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        
        if abs(self.loc_x - self.dest_x) <= 1. and abs(self.loc_y - self.dest_y) <= 1.: # arrived at destination
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            print("Arrived ", self.loc_x, self.loc_y)
        else:        
            t0 = rospy.Time.now().to_sec()
            # check whether toward destination
            if self.time_interval >= adjust_interval:

                dest_angle = np.arctan((self.dest_y - self.loc_y)/(self.dest_x - self.loc_x))

                vel_msg.linear.x = 0
                vel_msg.angular.z = sgn(dest_angle)
                self.velocity_publisher.publish(vel_msg)
                t1=rospy.Time.now().to_sec()
                self.curr_angle += vel_msg.angular.z * 2 * np.pi * (t1 - t0)    
                if abs(self.curr_angle - dest_angle) <= 0.2:  # 12 degree 
                    self.time_interval = 0  
            
            else:
                # move to avoid collision       
                if self.coll <= 0.9:
                    vel_msg.linear.x = 0.05
                else:
                    vel_msg.linear.x = 0    
                vel_msg.angular.z = sgn(self.steer)

                self.velocity_publisher.publish(vel_msg)     
                t1=rospy.Time.now().to_sec()
            
                # calculate current position
                self.curr_angle += vel_msg.angular.z * 2 * np.pi * (t1 - t0)
                forward_dis =  np.float(vel_msg.linear.x * (t1 - t0))
                self.loc_x += forward_dis * np.cos(np.float(self.curr_angle))
                self.loc_y += forward_dis * np.sin(np.float(self.curr_angle))        
                self.time_interval += t1 - t0
        
   
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