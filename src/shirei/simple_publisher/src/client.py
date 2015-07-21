#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
from std_srvs.srv import Empty

def r():
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        resp1 = reset()
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
        
r()