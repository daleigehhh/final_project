#!/usr/bin/env python
import numpy as np 
import sys
import rospy 
import cv2
import tf
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import PlanningSceneInterface

class image_processor:
    def __init__(self):
        self.flag = True
        self.image_pub1 = rospy.Publisher("table_detect", Image, queue_size=10)
        self.image_pub2 = rospy.Publisher("cube_detect", Image, queue_size=10)
        self.pose_pub = rospy.Publisher("pose_trans", PoseStamped, queue_size=10)
        self.image_sub = rospy.Subscriber("arm/camera/image_raw",Image,self.callback)
        self.bridge = CvBridge()
        self.scene = PlanningSceneInterface()

        rospy.init_node('vision_manager')
        br = tf.TransformBroadcaster()
        coor = tf.transformations.quaternion_from_euler(0, 0, -1.57075)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            br.sendTransform(
                (-0.75, 0.85, 1.05),
                coor,
                rospy.Time.now(),
                "table",
                "world"
            )
            rate.sleep()

    def callback(self, data):
        try:
            table_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.flag == True:
            left, right, top, bottom, table_width, table_length, graph1 = self.findTable(table_image)
            print('table length = %f' % table_length)
            print('table width = %f' % table_width)

            cube_image = image[left+1:right, top:bottom]
            position, cube_angle, graph2 = self.findCube(cube_image)

            conversion = (0.7/table_width + 1.5/table_length)/2

            cube_position = (conversion*(position[0]-2), conversion*(position[1]-2))

        
            rospy.loginfo('Pose of the cube:')
            print('position:')
            print(cube_position)

            print('angle:')
            print(cube_angle)         
        
            '''cv2.imshow("graph1", graph1)
            cv2.imshow("graph2", graph2)
            cv2.waitKey(0)
            cv2.destoryAllWindows()'''

            #q = tf.transformations.quaternion_from_euler(0, 0, cube_angle)
            pose_under_table_frame = PoseStamped()
            pose_under_table_frame.header.frame_id = "table"
            pose_under_table_frame.pose.position.x = cube_position[0]
            pose_under_table_frame.pose.position.y = cube_position[1]
            pose_under_table_frame.pose.position.z = 0.0
            '''pose_under_table_frame.pose.orientation.x = q[0]
            pose_under_table_frame.pose.orientation.y = q[1]
            pose_under_table_frame.pose.orientation.z = q[2]
            pose_under_table_frame.pose.orientation.w = q[3]'''

            pose_listener = tf.TransformListener()

            try:
                pose_listener.waitForTransform("world", "table", rospy.Time(0), rospy.Duration(0.2))
                pose_listener.lookupTransform("world", "table", rospy.Time(0))
                pose_under_world_frame = pose_listener.transformPose("world",  pose_under_table_frame)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                pose_under_world_frame = None
                print("failed to transform pose.")
                print(e)

            rospy.loginfo('Transform pose:')
            print(pose_under_world_frame)

            self.pose_pub.publish(pose_under_world_frame)
            self.flag = False

    def findTable(self, image):
        rows, cols, channels = image.shape
        b,g,r = cv2.split(image)
        for i in range(rows):
            for j in range(cols):
                if b[i,j] > 100 and g[i,j] >100:
                    image[i,j] = [255, 255, 255]
                else:
                    image[i,j] = [0, 0, 0]
        
        edges = cv2.Canny(image, 100, 100)

        values = np.where(edges == 255)

        left = np.min(values[0])
        right = np.max(values[0])
        top = np.min(values[1])
        bottom = np.max(values[1])

        length = right - left
        width = bottom - top

        #img = image[left+1:right-1, top+1:bottom-1]

        return left, right, top, bottom, length, width, edges

    def findCube(self, image):
        rows, cols, channels = image.shape
        b, g, r = cv2.split(image)
        for i in range(rows):
            for j in range(cols):
                if g[i,j] > 70:
                    image[i,j] = [255, 255, 255]                                      
                else: 
                    image[i,j] = [0, 0, 0]           

        edges = cv2.Canny(image, 100, 100) 
        for i in range(rows):
            edges[i,0] = 0
            
        print "graph's 0,0: %d" % edges[0,0]

        values = np.where(edges == 255)
       
        if any(values[0]):
            far_top_y = np.min(values[0])
            print('far_top_y is ')
            print(far_top_y)
            far_top_x_set = np.where(values[0] == far_top_y)
            far_top_x = values[1][far_top_x_set[0]][0]
            print('far_top_x is ')
            print(far_top_x)

            far_left_x = np.min(values[1])
            print('far_left_x is ')
            print(far_left_x)
            far_left_y_set = np.where(values[1] == far_left_x)
            far_left_y = values[0][far_left_y_set[0]][-1]
            print('far_left_y is ')
            print(far_left_y)

            triangle_x = far_left_x - far_top_x
            triangle_y = far_left_y - far_top_y

            angle = round(math.atan(triangle_x/triangle_y), 1)

            left = np.min(values[0])
            right = np.max(values[0])
            top = np.min(values[1])
            bottom = np.max(values[1])

            mid = ((left+right)/2, (top+bottom)/2)
            print('position is :')
            print(mid)
            return mid, angle, edges
        else:
            rospy.logwarn("No objects found, spawn objecs first!")
            sys.exit(1)

def main(args):
    ic = image_processor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shuting down")

if __name__ == '__main__':
    main(sys.argv)