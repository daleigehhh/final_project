#!/usr/bin/env python

import rospy
import sys
from rrbot.srv import *

def judge_client(done):
    rospy.wait_for_service("Judge")
    try:
        judgement = rospy.ServiceProxy("Judge", judge)
        resp1 = judgement(done)
        if resp1.success == True:
            print "grasp success"
        else:
            print "grasp failed"
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

done = True
judge_client(done)