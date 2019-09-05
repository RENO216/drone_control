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
import time



if __name__ == '__main__':
  pub_land = rospy.Publisher("ardrone/land", Empty, queue_size=10)
  rospy.init_node('land', anonymous=True, disable_signals=True)
  rate = rospy.Rate(10) # 10hz
  while True: 
    try:      
      pass
    except KeyboardInterrupt:
        pub_land.publish(Empty())
        rate.sleep()


