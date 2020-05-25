#!/usr/bin/env python
from rrbot.srv import judge, judgeResponse
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2

result_image = None

bridge = CvBridge()

def sub_callback(data):
    global result_image, bridge
    result_image = bridge.imgmsg_to_cv2(data, "bgr8")

image_sub = rospy.Subscriber("arm/camera2/image_raw", Image, sub_callback)


def handle_judge(req):
    rospy.loginfo("Test the result of grasping.")
    global result_image
    if req.judge == True:
        rows, cols, channels = result_image.shape
        b, g, r = cv2.split(result_image)
        for i in range(rows):
            for j in range(cols):
                if b[i,j] > 100 and g[i,j] < 50 and r[i,j] < 50:
                    return True
                    break
        return False
def judge_server():
    rospy.init_node("judge_node")
    s = rospy.Service("Judge", judge, handle_judge)
    print("Ready to judge the result of grasping")
    rospy.spin()

judge_server()
