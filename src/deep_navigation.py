#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import numpy as np
import six.moves.urllib as urllib
from std_msgs.msg import Float32
from drone_control.msg import filter_state
import sys
import os
import math


class deep_navigation:
    def __init__(self, dest_x=0., dest_y=0., adjust_interval=3.):
        self.steer_sub = rospy.Subscriber("steer", Float32, self.steer_update)
        self.coll_sub = rospy.Subscriber("coll", Float32, self.coll_update)
        self.pos_sub = rospy.Subscriber('/ardrone/predictedPose', filter_state, self.pos_update)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.alpha = 0.5
        self.beta = 0.7

        # original collision probability and steering direction
        self.coll = 1
        self.steer = 0

        # original position
        self.time_interval = 0.
        self.adjust_interval = adjust_interval

        # position & direction
        self.loc_x = 0.
        self.loc_y = 0.
        self.yaw = 0.
        self.curr_angle = 0.

        # destination
        self.dest_x = dest_x
        self.dest_y = dest_y

        # initialization state
        self.ini = True
        
    def pos_update(self, data):
        self.loc_x = data.x
        self.loc_y = data.y
        self.yaw = data.yaw

    def steer_update(self, data):
        self.steer = data.data

    def coll_update(self, data):
        self.coll = data.data
        self.move()



    def move(self):
        def sgn(x): return 1. if x > 0 else -1. if x < 0 else 0.
        print("time interval:", self.time_interval)

        pi = np.pi
        adjust_interval = self.adjust_interval
        vel_msg = Twist()
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # arrived at destination
        if abs(self.loc_x - self.dest_x) <= 0.1 and abs(self.loc_y - self.dest_y) <= 0.1:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            print("Arrived ", self.loc_x, self.loc_y)

        # during flying
        else:
            t0 = rospy.Time.now().to_sec() * 100 # in second

            # adjust direction towards destination
            # if self.time_interval >= adjust_interval:       
            #     self.ini = False
            #     vel_msg.linear.x = 0

            #     dest_angle = np.arctan((self.dest_y - self.loc_y)/(self.dest_x - self.loc_x))
            #     if self.dest_y - self.loc_y > 0 and self.dest_x - self.loc_x < 0:
            #         dest_angle += pi
            #     if self.dest_y - self.loc_y < 0 and self.dest_x - self.loc_x < 0:
            #         dest_angle -= pi

            #     print("Adjust!", self.loc_x, self.loc_y, "curren angle:", self.yaw, \
            #         "dest angle: ", dest_angle / pi * 180)
                
                
            #     if abs(self.yaw - dest_angle / pi * 180) <= 15:  
            #         print("-------------------------direction reset done!-------------------------------")
            #         self.time_interval = 0
            #         vel_msg.angular.z = 0
            #     else:
            #         vel_msg.angular.z = sgn(dest_angle - self.curr_angle)
            #     self.velocity_publisher.publish(vel_msg)
            #     t1 = rospy.Time.now().to_sec() * 1000

            #     # self.curr_angle += vel_msg.angular.z * 2 * pi * (t1 - t0)
            #     self.curr_angle = self.yaw / 180 * pi if self.yaw > 0 else (self.yaw + 360) / 180 * pi 

            # # move forward and avoid collision
            # else:     
            if self.time_interval >= adjust_interval:
                self.ini = False
            if self.ini == True:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
            else:
                if self.coll <= 0.8:
                    vel_msg.linear.x = 0.
                    vel_msg.angular.z = 0.
                else:
                    print("might coll:", self.coll)
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = sgn(self.steer) / 3
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec() * 100

            # # calculate current position
            self.curr_angle += vel_msg.angular.z * 2 * pi * (t1 - t0)
            # forward_dis = np.float(vel_msg.linear.x * (t1 - t0))
            # self.loc_x += forward_dis * \
            #     np.cos(np.float(self.curr_angle)) * 20  # in meter
            # self.loc_y += forward_dis * \
            #     np.sin(np.float(self.curr_angle)) * 20  # in meter
            self.time_interval += t1 - t0

def main(args):
    rospy.init_node('deep_navigation', anonymous=True)
    publand = rospy.Publisher("ardrone/land", Empty, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    dn = deep_navigation(dest_x=4., dest_y=-4.)
    try:
        # dn.move()
        rospy.spin()
    except KeyboardInterrupt:
        print("STOP TRIP")
        while not rospy.is_shutdown():
            publand.publish(Empty())
            rate.sleep()


if __name__ == '__main__':
    main(sys.argv)
