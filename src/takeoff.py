#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

def takeoff():
        pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10 )
        land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=1)
        rospy.init_node('takeoff', anonymous=True, disable_signals=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
          try:
            pub.publish(Empty())
            rate.sleep()
          except KeyboardInterrupt:   
            land_pub.publish(Empty())
            break

if __name__ == '__main__':
        takeoff()
