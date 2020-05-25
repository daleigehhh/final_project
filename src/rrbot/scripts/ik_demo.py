#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import *

class MoveItIkDemo:
    roll = input("roll: ")
    pitch = input("pitch: ")
    yaw = input("yaw: ")
    def __init__(self):

        #init move_group
        moveit_commander.roscpp_initialize(sys.argv)

        #init ROS node
        rospy.init_node('moveit_ik_demo')

        #init arm group
        arm = moveit_commander.MoveGroupCommander('arm')

        #get the eef link
        end_effector_link = arm.get_end_effector_link()

        #set goal position coordinate
        reference_frame = 'base'
        arm.set_pose_reference_frame(reference_frame)

        #enable replanning
        arm.allow_replanning(True)

        

        #set error tolerance
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)

        #control the arm back to the orientation
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)

        #set the goal position of the arm,xyz
        #set the goal pose,xyzw
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = -0.3
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0.7
        target_pose.pose.orientation.x = sin(self.roll*pi/360)*cos(self.yaw*pi/360)*cos(self.pitch*pi/360) - cos(self.roll*pi/360)*sin(self.yaw*pi/360)*sin(self.pitch*pi/360)
        target_pose.pose.orientation.y = cos(self.roll*pi/360)*sin(self.yaw*pi/360)*cos(self.pitch*pi/360) + sin(self.roll*pi/360)*cos(self.yaw*pi/360)*sin(self.pitch*pi/360)
        target_pose.pose.orientation.z = cos(self.roll*pi/360)*cos(self.yaw*pi/360)*sin(self.pitch*pi/360) - sin(self.roll*pi/360)*sin(self.yaw*pi/360)*cos(self.pitch*pi/360)
        target_pose.pose.orientation.w = cos(self.roll*pi/360)*cos(self.yaw*pi/360)*cos(self.pitch*pi/360) + sin(self.roll*pi/360)*sin(self.yaw*pi/360)*sin(self.pitch*pi/360)


        #set current condition as beginning 
        arm.set_start_state_to_current_state()

        #set goal pose
        arm.set_pose_target(target_pose, end_effector_link)

        #set motion plan
        traj = arm.plan()

        #control
        arm.execute(traj)
        rospy.sleep(1)

        #back to home
        arm.set_named_target('place')
        arm.go()

        #shutdown
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()
    