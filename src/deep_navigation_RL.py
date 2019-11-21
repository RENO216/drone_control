#!/usr/bin/env python
"""
project script for course RL
This script collects dataset for RL algorithm
dataset form is as described from line 79~85
"""
import sys, os, math, csv, time
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
                 k_near = 2, # the nearest obstacles to the drone
                 col_data_mode = True # whether is collecting data samples for RL training
                 ):
        
        global curtpath

        self.pos_sub = rospy.Subscriber('/ardrone/predictedPose', filter_state, self.pos_update)
        if not col_data_mode:
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
        self.adjust_interval = adjust_interval # the time interval of adjustment towards the destination
        self.time_multi = 1000 # time multiplier from simulation to the reality
        self.destination_error = destination_error # the approximation error allowed for checking whether arrived at the destination

        # position & direction
        self.loc_x = 0.
        self.loc_y = 0.
        self.yaw = 0.
        self.curr_angle = 0. # absolute yaw
        self.forward_speed = forward_speed
        self.dis_accumu = 0. # Accumulated flying distance 

        # destination
        self.dest_x = dest_x
        self.dest_y = dest_y

        # whether initialization
        # This step is necessary since the drone needs time to settle up in the initial state, o.w. it will not be stable while taking off
        self.ini = True
        
        if col_data_mode:

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
            # |angle and distance between the drone and the obstacle (\in R^{1 * (2*k)}) | velocity | steering | position x | position y | distance till now | # trips |
            # --------------------------------------------------------------------------------------  
            # action space: 
            # velocity +, 0, -; steering left, forward, right. In total 9 actions      

            self.temp_record = np.zeros([1, 2 * k + 4]]) # record state
            self.reward = 0
            self.write_path = str(curtpath + '/traj.csv')
        


    def pos_update(self, data):
        """
        Update drone parameters
        """
        global pi
        t0 = rospy.Time.now().to_sec() * self.time_multi

        if self.time_interval >= 3:
            self.ini =  False

        dis_add = np.sqrt(pow(data.x - self.loc_x, 2) + pow(data.x - self.loc_x, 2))
        self.loc_x = data.x
        self.loc_y = data.y
        self.yaw = data.yaw
        self.velo = data.dx
        self.droneState = data.droneState
        self.dis_accumu += dis_add

        # IF it's in the data collection mode
        if col_data_mode and not self.ini:
            """
            TODO: 
            1. update R_{t+1}
            2. update parameters relevant to obstacles: def params_obstacle()
            """

            # update **current** position parameters: s = s'
            self.temp_record[-6] = self.velo
            self.temp_record[-5] = self.yaw / 180 * pi # use \pi form
            self.temp_record[-4] = self.loc_x
            self.temp_record[-3] = self.loc_y
            self.temp_record[-2] = self.dis_accumu
            # choose action randomly
            ac = np.random.randint(0, 9)
            action_result(action = ac)
            # move
            self.moveRL()
        
            if self.time_interval < self.adjust_interval: # since when time interval exceeds, readjust towards the destination
                """
                TODO: write to csv
                """
                pass
        
        t1 = rospy.Time.now().to_sec() * self.time_multi
        self.time_interval += t1 - t0


    """
    function pos_act_update() and move() are combined for the situation col_data_mode = False
    """
    def pos_act_update(self, data):
        """
        Used when col_data_mode is False, i.e., while collecting data samples this function wouldn't be called
        """
        self.steer = data.steer
        self.coll = data.coll_prob
        self.disToObj = data.dis_to_obj
        self.move()

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
            self.temp_record[-1] += 1 # complete one trip, trip +1

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

            # move forward and avoid collision
            else:
                t0 = rospy.Time.now().to_sec() * self.time_multi
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
    

    """
    function action_result() and moveRL() are combined for the situation col_data_mode = True
    """
    def action_result(self, action):
        """
        # velocity +, 0, -; steering left, forward, right. In total 9 actions
        This function is used if col_data_mode = True
        """
        
        if action // 3 == 0:
            self.forward_speed += 0.1 if self.forward_speed < 1 else 0 # increase speed (max speed: 1)
        elif action // 3 == 2:
            self.forward_speed -= 0.1 if self.forward_speed > 0.1 else 0 # decrease speed (min speed: 0.1)
        
        if action % 3 == 0:
            self.steer = 1  # left
        elif action % 3 == 1:
            self.steer = 0  # forward
        elif action % 3 == 2:
            self.steer = -1 # right

    def moveRL(self):

        def sgn(x): return 1. if x > 0 else -1. if x < 0 else 0.
        global pi
        adjust_interval = self.adjust_interval

        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.curr_angle = self.yaw / 180 * pi

        # take actions when it's not initial state and the drone is flying rather than landing on the ground
        if not self.ini and self.droneState != 3 and self.droneState != 4 and self.droneState != 7:
            # adjust towards the destination o.w. we don't know the drone will fly to where
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

            else: # simply randomly flying
                vel_msg.linear.x = self.forward_speed
                vel_msg.angular.z = self.steer
            self.velocity_publisher.publish(vel_msg)

    def get_reward(self):
        """
        TODO: reward gotten when 
        1. get to the destination
        2. collision with obstacles
        """
        # arrived at destination
        if abs(self.loc_x - self.dest_x) <= self.destination_error and abs(self.loc_y - self.dest_y) <= self.destination_error:
            self.reward = 100 / (abs(self.loc_x - self.dest_x) + abs(self.loc_y - self.dest_y))
            # publish movements
            
            self.temp_record[-1] += 1 # complete one trip, trip +1

        # collision with obstacles
        elif:
            pass

    def params_obstacle(self):
        """
        TODO: state data considering the parameters relevant to obstacles
        1. get the angles of the k nearest obstacles
        2. get the distance to the k nearest obstacle
        """
        pass



    """
    other utils
    """

    def write_to_cvs(filename="default", mat=None, csv_mode='a+'):
        with open(str(filename + '.csv'), csv_mode) as file_test:
            for i in range(mat.shape[0]):
                writer = csv.writer(file_test)
                writer.writerow(mat[i, :])
    
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
