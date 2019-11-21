#!/usr/bin/env python
"""
This script uses object detection to avoid collisions.
"""
import sys, os, math, csv
import rospy
from std_msgs.msg import String, Empty, Float32
from drone_control.msg import filter_state, pos_status # message types
from geometry_msgs.msg import Twist
import numpy as np
import pandas as pd
import six.moves.urllib as urllib

# global parameters
pi = np.pi
curtpath = sys.path[0]

class deep_navigation:
    """
    Script for drone navigation using tensorflow object detection API
    Problems:
        Can't detect objects when getting too close to an object
        Not smooth control
    """
    def __init__(self,
                 dest_x=0.,
                 dest_y=0.,
                 adjust_interval=1.,
                 forward_speed=0.5,
                 destination_error=0.5,
                 k_near = 2 # the nearest obstacles to the drone
                 ):
        
        global curtpath

        self.pos_sub = rospy.Subscriber('/ardrone/predictedPose', filter_state, self.pos_update)
        self.pos_act_sub = rospy.Subscriber('pos_act_params', pos_status, self.pos_act_update)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.alpha = 0.5
        self.beta = 0.7

        # original collision probability and steering direction
        self.coll = 1
        self.steer = 0
        self.line_z = 0

        # original time interval
        self.time_interval = 0. # accumulated time interval
        self.adjust_interval = adjust_interval # adjustment frequency
        self.time_multi = 1000 # time multiplier from simulation to the reality
        self.destination_error = destination_error # the approximation error allowed for checking whether arrived at the destination

        # position & direction
        self.loc_x = 0.
        self.loc_y = 0.
        self.yaw = 0.
        self.curr_angle = 0. # absolute yaw
        self.forward_speed = forward_speed
        self.dis_accumu = 0. # flied accumulated distance 

        # destination
        self.dest_x = dest_x
        self.dest_y = dest_y

        # whether initialization
        self.ini = True
        
        # get environment obstacles coordinates
        self.envob = np.zeros([1,2])
        envFile = open(str(curtpath + '/env.csv'), "r")
        reader = csv.reader(envFile)
        for item in reader:
            self.envob = np.vstack([self.envob, np.array(item).astype(float)])
        self.envob = self.envob[1:, :]

        # record performance data
        # state:
        # --------------------------------------------------------------------------------------
        # |angle and distance between the drone and the obstacle (\in R^{1 * (2*k)}) | position x | position y | distance till now | # trips |
        # --------------------------------------------------------------------------------------  
        # action space: 
        # velocity +, 0, -; steering left, forward, right. In total 9 actions

        self.temp_record = np.zeros([1, 2 * k + 4]]) # record state
        self.write_path = str(curtpath + '/traj.csv')
        


    def pos_update(self, data):
        """
        Update drone position
        """
        dis_add = np.sqrt(pow(data.x - self.loc_x, 2) + pow(data.x - self.loc_x, 2))
        self.loc_x = data.x
        self.loc_y = data.y
        self.yaw = data.yaw
        self.droneState = data.droneState
        self.dis_accumu += dis_add

        self.temp_record[-4] = self.loc_x
        self.temp_record[-3] = self.loc_y
        self.temp_record[-2] = self.dis_accumu

    def pos_act_update(self, data):
        self.steer = data.steer
        self.coll = data.coll_prob
        self.disToObj = data.dis_to_obj
        self.move()

    def write_to_cvs(filename="default", mat=None, csv_mode='a+'):
        with open(str(filename + '.csv'), csv_mode) as file_test:
            for i in range(mat.shape[0]):
                writer = csv.writer(file_test)
                writer.writerow(mat[i, :])
    
    def move(self):

        def sgn(x): return 1. if x > 0 else -1. if x < 0 else 0.
        global pi

        adjust_interval = self.adjust_interval
        vel_msg = Twist()
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        self.curr_angle = self.yaw / 180 * pi

        # arrived at destination
        if abs(self.loc_x - self.dest_x) <= self.destination_error and abs(self.loc_y - self.dest_y) <= self.destination_error:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            print("Arrived ", self.loc_x, self.loc_y)
            self.temp_record[-1] += 1 

            # redefine a destination
            self.dest_x = np.random.randint(-3,5)
            self.dest_y = np.random.randint(-3,5)

        # whether hovering/flying
        elif self.droneState != 3 and self.droneState != 4 and self.droneState != 7:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            # print("Not Flying")

        # during flying
        else:
            t0 = rospy.Time.now().to_sec() * self.time_multi  # in second

            # adjust direction towards destination
            if self.time_interval >= adjust_interval:
                self.ini = False
                vel_msg.linear.x = 0 # set forward speed to 0
                dest_angle = np.arctan((self.dest_y - self.loc_y)/(self.dest_x - self.loc_x))

                if self.dest_x - self.loc_x < 0:
                    dest_angle = -(pi/2 + dest_angle)
                if self.dest_x - self.loc_x > 0:
                    dest_angle = pi/2 - dest_angle

                # print("Adjust!", self.loc_x, self.loc_y, "curren angle:", self.yaw, "dest angle: ", dest_angle / pi * 180)

                if abs(self.yaw - dest_angle / pi * 180) <= 15:
                    print("-------------------------direction reset done!-------------------------------")
                    self.time_interval = 0
                    vel_msg.angular.z = 0
                else:
                    # clockwilse z-;
                    vel_msg.angular.z = - sgn(dest_angle - self.curr_angle)
                self.velocity_publisher.publish(vel_msg)
                t1 = rospy.Time.now().to_sec() * self.time_multi

            # move forward and avoid collision
            else:
                if self.ini == True:
                    print(str('-------------------------Initiation-------------------------' + str(self.time_interval)))
                    vel_msg.linear.x = 0
                    vel_msg.angular.z = 0
                else:
                    if self.coll <= 0.9:
                        vel_msg.linear.x = self.forward_speed
                        vel_msg.angular.z = 0
                    else:
                        vel_msg.linear.x = 0.
                        vel_msg.angular.z = sgn(self.steer)
                
                self.velocity_publisher.publish(vel_msg)

                t1 = rospy.Time.now().to_sec() * self.time_multi
                self.time_interval += t1 - t0

        self.write_to_cvs()
        
    
    def write_to_csv(self):
        """
        TODO: 
        """
        pass


def main(args):
    rospy.init_node('deep_navigation', anonymous=True)
    rate = rospy.Rate(5)  # 5hz
    dn = deep_navigation(dest_x=3.,
                         dest_y=3.,
                         forward_speed=0.5,
                         destination_error=0.2)
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
