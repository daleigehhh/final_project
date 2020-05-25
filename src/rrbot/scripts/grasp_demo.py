#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import actionlib
import moveit_commander
import control_msgs.msg
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor, MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose
from tf import transformations
from rrbot.srv import *


class MoveItObstaclesDemo:
    def __init__(self):
        rospy.init_node('grasp_demo')
        rospy.Subscriber('pose_trans', PoseStamped, self.callback)
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size = 10)
        rospy.spin()

    def callback(self, data):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
        
        # 等待场景准备就绪
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        gripper = MoveGroupCommander('gripper')
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
       
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base'
        arm.set_pose_reference_frame(reference_frame)
        
        # 设置每次运动规划的时间限制：5s
        arm.set_planning_time(5)
        
        # 设置场景物体的名称
        table_id = 'table'
        
        # 控制机械臂先回到初始化位置
        '''arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)'''
        
        # 设置桌面的高度
        table_ground = 0.6
        
        # 设置table、box1和box2的三维尺寸
        table_size = [1.5, 0.7, 0.01]
        
        # 将三个物体加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.5
        table_pose.pose.position.z = 0.825
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        rospy.sleep(1)
        
        # 将桌子设置成红色，两个box设置成橙色
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        
        # 将场景中的颜色设置发布
        self.sendColors()   

        # 设置机械臂的运动目标位置，进行避障规划
        '''orient = transformations.quaternion_from_euler(0, 1.57075, 0)
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = -0.1
        target_pose.pose.position.y = 0.5
        target_pose.pose.position.z = 1.05
        target_pose.pose.orientation.x = orient[0]
        target_pose.pose.orientation.y = orient[1]
        target_pose.pose.orientation.z = orient[2]
        target_pose.pose.orientation.w = orient[3]'''
        
        orient = transformations.quaternion_from_euler(0, 1.57075, 0)
        target_pose = data
        target_pose.pose.orientation.x = orient[0]
        target_pose.pose.orientation.y = orient[1]
        target_pose.pose.orientation.z = orient[2]
        target_pose.pose.orientation.w = orient[3]

        # 控制机械臂运动到目标位置
        self.gripper_control(0)
        result = None
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()

        rospy.loginfo('Start grasping')
        target_pose.pose.position.z -= 0.05
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()

        self.gripper_control(0.36)
        '''arm.attach_object( box2_id)
        arm.set_named_target('home')
        arm.go()'''

        arm.set_named_target('home')
        arm.go()
        self.gripper_control(0)   
        rospy.loginfo('Grasping done')
        rospy.sleep(2)
        judge_client(True)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
        
    # 设置场景物体的颜色
    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

    def gripper_control(self, position):
         # Create an action client
        client = actionlib.SimpleActionClient(
            '/arm/gripper_controller/gripper_cmd',  # namespace of the action topics
            control_msgs.msg.GripperCommandAction # action type
        )
            
        # Wait until the action server has been started and is listening for goals
        client.wait_for_server()
        
        # Create a goal to send (to the action server)
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = position   # From 0.0 to 0.8
        goal.command.max_effort = -1  # Do not limit the effort
        client.send_goal(goal)
        client.wait_for_result()
        rospy.sleep(1)

def judge_client(done):
    rospy.wait_for_service("Judge")
    try:
        judgement = rospy.ServiceProxy("Judge", judge)
        resp1 = judgement(done)
        if resp1.success == True:
            rospy.loginfo("==========grasp success==========")
        else:
            rospy.loginfo("==========grasp failed==========")
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    try:
        MoveItObstaclesDemo()
    except KeyboardInterrupt:
        raise
    

    
