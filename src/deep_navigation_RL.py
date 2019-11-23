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
                 adjust_interval=10.,
                 forward_speed=0.5,
                 destination_error=0.5,
                 k_near = 2, # the nearest obstacles to the drone
                 col_data_mode = True, # whether is collecting data samples for RL training
                 k = 3 # state parameters of k nearest obstacles
                 ):
        
        global curtpath
        self.col_data_mode = col_data_mode

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
        
        if self.col_data_mode:

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
            # |distance and anglesbetween the drone and the obstacle (\in R^{1 * (2*k)}) | velocity | steering | position x | position y | distance till now | # trips |
            # --------------------------------------------------------------------------------------  
            # action space: 
            # velocity +, 0, -; steering left, forward, right. In total 9 actions      

            self.ac = 0
            self.reward = 0
            self.no_trip = 0
            self.write_path = str(curtpath + '/traj.csv')
            self.w_ini = True
            
        


    def pos_update(self, data):
        """
        Update drone parameters
        """
        global pi

        # check current situation (taking off? landing? etc.)
        Sig = data.droneState
        if Sig == 3 or Sig == 4: # hovering or flying
            self.ini =  False
        elif Sig == 2 or Sig == 8:
            self.ini =  True

        # the coordinates in "drone" space and "env" space are different, the transformation is shown in ./env_explain.pdf
        dis_add = np.sqrt(pow(data.y - self.loc_x, 2) + pow(-data.x - self.loc_y, 2))
        self.loc_x = data.y
        self.loc_y = - data.x
        self.yaw = data.yaw
        self.velo = data.dx
        self.droneState = data.droneState
        self.dis_accumu += dis_add

        # IF it's in the data collection mode
        if self.col_data_mode and not self.ini:
            """
            1. update R_{t+1}
            2. write to csv s, a, R_{t+1}
            3. update current state s <- s'
            4. choose action and move to s'
            """         
            k = 3
            # update self.reward
            self.get_reward()
            # write to the csv 
            # if self.time_interval < self.adjust_interval: # since when time interval exceeds, readjust towards the destination
                # write to csv

            if not self.w_ini:
                record_row = np.hstack([np.around(self.temp_record[0,:], 4), self.ac, self.reward])       
                print(record_row)         
                with open(self.write_path, 'a+') as file_test:
                    writer = csv.writer(file_test)
                    writer.writerow(record_row)
            
            if self.w_ini:
                self.w_ini = False

            # update **current** position parameters: s <- s'       
            dists_k, angles_k = self.cal_obs_state(k = k)
            self.temp_record = np.hstack([dists_k, angles_k, \
                np.array([self.velo, self.yaw, self.loc_x, self.loc_y, self.dis_accumu, self.no_trip]).reshape([1,-1])])
            # choose action randomly
            self.ac = np.random.randint(0, 9)
            self.action_result(action = self.ac)
            # move
            self.moveRL()
        
                            
        # 
        # 
        # print("time inter:", self.time_interval)


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
            self.steer = 10  # left
        elif action % 3 == 1:
            self.steer = 0  # forward
        elif action % 3 == 2:
            self.steer = -10 # right
        
    def moveRL(self):

        def sgn(x): return 1. if x > 0 else -1. if x < 0 else 0.
        global pi
        adjust_interval = self.adjust_interval
        t0 = rospy.Time.now().to_sec() * self.time_multi

        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.curr_angle = self.yaw / 180 * pi

        # take actions when it's not initial state and the drone is flying rather than landing on the ground
        if not self.ini:
            # # adjust towards the destination o.w. we don't know the drone will fly to where
            # if self.time_interval >= adjust_interval:
            #     self.ini = False
            #     vel_msg.linear.x = 0 # set forward speed to 0
            #     dest_angle = np.arctan((self.dest_y - self.loc_y)/(self.dest_x - self.loc_x))

            #     if self.dest_x - self.loc_x < 0:
            #         dest_angle = -(pi/2 + dest_angle)
            #     if self.dest_x - self.loc_x > 0:
            #         dest_angle = pi/2 - dest_angle

            #     # print("Adjust!", self.loc_x, self.loc_y, "curren angle:", self.yaw, "dest angle: ", dest_angle / pi * 180)

            #     if abs(self.yaw - dest_angle / pi * 180) <= 15:
            #         # print("-------------------------direction reset done!-------------------------------")
            #         self.time_interval = 0
            #         vel_msg.angular.z = 0
            #     else:
            #         # clockwilse z-;
            #         vel_msg.angular.z = - sgn(dest_angle - self.curr_angle)

            # else: # simply randomly flying
            vel_msg.linear.x = self.forward_speed
            vel_msg.angular.z = self.steer
            self.velocity_publisher.publish(vel_msg)

            t1 = rospy.Time.now().to_sec() * self.time_multi
            self.time_interval += t1 - t0

    def get_reward(self):
        """
        reward gotten when 
        1. get to the destination, reward = 100 / error 
        2. collision with obstacles (distance within 0.4), reward = - 10
        """

        self.reward = -1

        # arrived at destination, reward = 100 / error
        error_x = abs(self.loc_x - self.dest_x)
        error_y = abs(self.loc_y - self.dest_y)
        if error_x <= self.destination_error and error_y <= self.destination_error:
            self.reward = 100 / (error_x + error_y)            
            self.no_trip += 1 # complete one trip, trip +1

        # collision with obstacles
        dists, _ = self.params_obstacle()
        if min(dists) <= 0.8:
            self.reward = - 10     

    def params_obstacle(self):    
        global pi

        # distance to those obstacles
        dists_x = self.envob[:, 0] - self.loc_x
        dists_y = self.envob[:, 1] - self.loc_y
        dists = np.sqrt(dists_x * dists_x + dists_y * dists_y)

        # angles to those obstacles
        ob_xs = self.envob[:, 0]
        ob_ys = self.envob[:, 1]
        angles = np.arctan((ob_ys - self.loc_y)/(ob_xs - self.loc_x))
        # adjust angles
        # np.arctan(1) = pi/4, np.arctan(-1) = -pi/4, thus np.arctan range from (-pi/2, pi/2)
        # according to the angle illustration in ./env_explain.pdf
        ind_adj = ob_ys - self.loc_y < 0
        angles[ind_adj] = pi/2 + angles[ind_adj]
        ind_adj = ob_ys - self.loc_y > 0
        angles[ind_adj] = - angles[ind_adj]
        return dists, angles

    def cal_obs_state(self, k = 3):
        """
        state data of the parameters relevant to obstacles
        1. get the angles of the k nearest obstacles
        2. get the distance to the k nearest obstacle
        """

        dists, angles = self.params_obstacle() # dist: 8 * 1 column, angles 8 * 1 column
        knear_ind = np.argpartition(dists, k)
        knear_ind = knear_ind[:k]

        dists_k = np.reshape(dists[knear_ind], [1, k])
        angles_k = np.reshape(angles[knear_ind], [1, k])

        return dists_k, angles_k # both are 1 * k array
        


    """
    other utils
    """

    def write_to_cvs(filename="default", mat=None, csv_mode='a+'):
        with open(str(filename + '.csv'), csv_mode) as file_test:
            for i in range(mat.shape[0]):
                writer = csv.writer(file_test)
                writer.writerow(mat[i, :])



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
