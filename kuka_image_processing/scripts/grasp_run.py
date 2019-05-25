#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import tf.transformations
import moveit_commander
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from image_segment.msg import target
from moveit_msgs.msg import Grasp, MoveItErrorCodes

REFERENCE_FRAME = 'base_link'
GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'
GRIPPER_FRAME = 'link6'
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.455
TARGET_NAME = 'unit_box_0'

# 初始化需要使用move group控制的机械臂中的arm group
arm = MoveGroupCommander(GROUP_NAME_ARM)


class MoveItPickAndPlace:
    def __init__(self, px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, w=1.0):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化场景对象
        scene = PlanningSceneInterface()
        scene.remove_world_object(TARGET_NAME)

        # 创建一个发布抓取姿态的发布者
        # self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)

        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame(REFERENCE_FRAME)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.005)
        gripper.set_goal_joint_tolerance(0.01)

        # 设置每次运动规划的时间限制：5s
        arm.set_planning_time(5)

        # 控制夹爪张开
        gripper.set_joint_value_target({"gripper_finger1_joint": GRIPPER_OPEN})
        gripper.go()
        rospy.sleep(1)

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
        target_pose = Pose()
        target_pose.position.x = px
        target_pose.position.y = py
        target_pose.position.z = pz + 0.10
        par = ox
        ox = 0
        (r, p, y) = tf.transformations.euler_from_quaternion((w, ox, oy, oz))
        (w, ox, oy, oz) = tf.transformations.quaternion_from_euler(
            par, p - np.pi / 2,  0)

        target_pose.orientation.x = ox
        target_pose.orientation.y = oy
        target_pose.orientation.z = oz
        target_pose.orientation.w = w

        arm.set_pose_target(target_pose, end_effector_link)

        # 规划运动路径
        traj = arm.plan()
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(3)

        arm.shift_pose_target(2, -0.06, end_effector_link)
        arm.go()
        rospy.sleep(1)

        # 闭合
        gripper.set_joint_value_target(
            {"gripper_finger1_joint": GRIPPER_CLOSED})
        gripper.go()
        rospy.sleep(2)

        arm.set_joint_value_target({"joint6": 5.42})
        arm.go()
        rospy.sleep(2)


        # 控制机械臂到目标位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)

        # arm.set_pose_target(target_pose, end_effector_link)

        # # 规划运动路径
        # traj = arm.plan()
        # # 按照规划的运动路径控制机械臂运动
        # arm.execute(traj)
        # arm.shift_pose_target(2, -0.15, end_effector_link)
        # arm.go()
        # rospy.sleep(3)

        # 控制夹爪回到张开的状态
        gripper.set_joint_value_target({"gripper_finger1_joint": GRIPPER_OPEN})
        gripper.go()
        rospy.sleep(1)

        # # 控制机械臂回到初始化位置
        # arm.set_named_target('home')
        # arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


class PosSubscriber:
    def __init__(self):
        self.px = 0
        self.py = 0
        self.pz = 0
        self.ox = 0
        self.oy = 0
        self.oz = 0
        self.w = 0

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)

        rospy.Subscriber('/target_world_coodinate', target,
                         self.callback, queue_size=1, buff_size=52400)

    def callback(self, data):
        self.px = data.x
        self.py = data.y
        self.pz = data.z
        print([data.x, data.y, data.z])
        self.ox = data.angle
        self.oy = 0
        self.oz = 0
        self.w = 0
        MoveItPickAndPlace(self.px, self.py, self.pz,
                           self.ox, self.oy, self.oz, self.w)


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('moveit_pick_and_place_demo', anonymous=True)

    PosSubscriber()

    rospy.spin()
