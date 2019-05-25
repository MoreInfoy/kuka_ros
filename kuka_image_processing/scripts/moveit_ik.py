#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import tf.transformations
import moveit_commander
import numpy as np
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from image_segment.msg import target

# from moveit_msgs.msg import PlanningScene, ObjectColor
# from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from tf.transformations import quaternion_from_euler
# from copy import deepcopy

BOX_ID = 'box'
BOX_SIZE = [0.05, 0.092474, 0.05]
MODEL_NAME = 'unit_box_0'
REFERENCE_FRAME = 'base_link'
GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'
GRIPPER_FRAME = 'link6'
TARGET_ID = 'box'
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.3


class BoxInGazeboPosePub:
    def __init__(self, px=0.0, py=0.0, pz=0.0, w=1.0):
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        # self.scene = PlanningSceneInterface()
    
        self.pose_msg = ModelState()
        self.pose_msg.model_name = MODEL_NAME
        self.pose_msg.reference_frame = REFERENCE_FRAME
        self.pose_msg.pose.position.x = px
        self.pose_msg.pose.position.y = py
        self.pose_msg.pose.position.z = pz
        self.pose_msg.pose.orientation.w = w

        # self.box_pose = PoseStamped()
        # self.box_pose.header.frame_id = REFERENCE_FRAME
        # self.box_pose.pose.position.x = px
        # self.box_pose.pose.position.y = py
        # self.box_pose.pose.position.z = pz
        # self.box_pose.pose.orientation.w = w

        self.publisher()

    def publisher(self):
        rate = rospy.Rate(1)
        theta = 0
        # self.scene.remove_world_object(BOX_ID)
        while not rospy.is_shutdown():
            # x = 0.811988378093 + 0.15 * np.cos(theta)
            # y = -0.176546923428 + 0.15 * np.sin(theta)
            # self.pose_msg.pose.position.x = x
            # self.pose_msg.pose.position.y = y
            # self.box_pose.pose.position.x = x
            # self.box_pose.pose.position.y = y
            self.pub.publish(self.pose_msg)
            # self.scene.add_box(BOX_ID, self.box_pose, BOX_SIZE)
            # theta += np.pi / 30
            rate.sleep()
            theta += 1
            # print(theta)
            if theta > 10:
                break


class MoveItPickAndPlace:
    def __init__(self, px=0.0, py=0.0, pz=0.0, ox=0.0, oy=0.0, oz=0.0, w=1.0):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化场景对象
        # scene = PlanningSceneInterface()

        # 创建一个发布抓取姿态的发布者
        # self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander(GROUP_NAME_ARM)

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

        # 设置pick和place阶段的最大尝试次数
        # max_pick_attempts = 5
        # max_place_attempts = 5
        # rospy.sleep(2)

        # 移除场景中之前与机器臂绑定的物体
        # scene.remove_attached_object(GRIPPER_FRAME, TARGET_ID)
        # rospy.sleep(1)

        # 控制夹爪张开
        gripper.set_joint_value_target({"gripper_finger1_joint": GRIPPER_OPEN})
        gripper.go()
        rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)

        # 设置支持的外观
        # arm.set_support_surface_name(TARGET_ID)

        # 设置一个place阶段需要放置物体的目标位置
        # place_pose = PoseStamped()
        # place_pose.header.frame_id = REFERENCE_FRAME
        # place_pose.pose.position.x = -0.03
        # place_pose.pose.position.y = -0.2
        # place_pose.pose.position.z = 0.505
        # place_pose.pose.orientation.w = 1.0

        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = px
        target_pose.pose.position.y = py
        target_pose.pose.position.z = pz + 0.2
        (r, p, y) = tf.transformations.euler_from_quaternion((w, ox, oy, oz))
        (w, ox, oy, oz) = tf.transformations.quaternion_from_euler(
            r, p - np.pi / 2, y - np.pi / 2)
        target_pose.pose.orientation.x = ox
        target_pose.pose.orientation.y = oy
        target_pose.pose.orientation.z = oz
        target_pose.pose.orientation.w = w

        # 将目标位置设置为机器人的抓取目标位置
        # grasp_pose = target_pose

        # 生成抓取姿态
        # grasps = self.make_grasps(grasp_pose, [TARGET_ID])
        #
        # # 将抓取姿态发布，可以在rviz中显示
        # for grasp in grasps:
        #     self.gripper_pose_pub.publish(grasp.grasp_pose)
        #     rospy.sleep(0.2)
        #
        # # 追踪抓取成功与否，以及抓取的尝试次数
        # result = None
        # n_attempts = 0
        #
        # # 重复尝试抓取，直道成功或者超多最大尝试次数
        # while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
        #     # grasp = grasps[n_attempts]
        #     n_attempts += 1
        #     rospy.loginfo("Pick attempt: " + str(n_attempts))
        #     result = arm.pick(TARGET_ID, grasps)
        #     rospy.sleep(0.2)
        #
        # # 如果pick成功，则进入place阶段
        # if result == MoveItErrorCodes.SUCCESS:
        #     result = None
        #     n_attempts = 0
        #
        #     # 生成放置姿态
        #     places = self.make_places(place_pose)
        #
        #     # 重复尝试放置，直道成功或者超多最大尝试次数
        #     while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
        #         n_attempts += 1
        #         rospy.loginfo("Place attempt: " + str(n_attempts))
        #         for place in places:
        #             result = arm.place(TARGET_ID, place)
        #             if result == MoveItErrorCodes.SUCCESS:
        #                 break
        #         rospy.sleep(0.2)
        #
        #     if result != MoveItErrorCodes.SUCCESS:
        #         rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        # else:
        #     rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        # arm.set_joint_value_target({"joint4": 2})
        # arm.go()
        # rospy.sleep(3)
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)

        # 规划运动路径
        traj = arm.plan()
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(3)

        # 控制机械臂终端平移
        arm.shift_pose_target(2, -0.15, end_effector_link)
        arm.go()
        rospy.sleep(1)

        # 控制机械臂终端反向旋转90度
        # arm.shift_pose_target(3, -1.57, end_effector_link)
        # arm.go()
        # rospy.sleep(1)

        gripper.set_joint_value_target({"gripper_finger1_joint": GRIPPER_CLOSED})
        gripper.go()
        rospy.sleep(4)

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        # 控制机械臂到目标位置
        arm.set_named_target('target')
        arm.go()

        # 控制夹爪回到张开的状态
        gripper.set_joint_value_target({"gripper_finger1_joint": GRIPPER_OPEN})
        gripper.go()
        rospy.sleep(1)

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # # 创建夹爪的姿态数据JointTrajectory
    # @staticmethod
    # def make_gripper_posture(joint_positions):
    #     # 初始化夹爪的关节运动轨迹
    #     t = JointTrajectory()
    #
    #     # 设置夹爪的关节名称
    #     t.joint_names = ['gripper_finger1_joint']
    #
    #     # 初始化关节轨迹点
    #     tp = JointTrajectoryPoint()
    #
    #     # 将输入的关节位置作为一个目标轨迹点
    #     tp.positions = joint_positions
    #
    #     # 设置夹爪的力度
    #     tp.effort = [2.0]
    #
    #     # 设置运动时间
    #     tp.time_from_start = rospy.Duration(1.0)
    #
    #     # 将目标轨迹点加入到运动轨迹中
    #     t.points.append(tp)
    #
    #     # 返回夹爪的关节运动轨迹
    #     return t
    #
    # # 使用给定向量创建夹爪的translation结构
    # @staticmethod
    # def make_gripper_translation(min_dist, desired, vector):
    #     # 初始化translation对象
    #     g = GripperTranslation()
    #
    #     # 设置方向向量
    #     g.direction.vector.x = vector[0]
    #     g.direction.vector.y = vector[1]
    #     g.direction.vector.z = vector[2]
    #
    #     # 设置参考坐标系
    #     g.direction.header.frame_id = REFERENCE_FRAME
    #
    #     # 设置最小和期望的距离
    #     g.min_distance = min_dist
    #     g.desired_distance = desired
    #
    #     return g
    #
    # # 创建一个允许的的抓取姿态列表
    # def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
    #     # 初始化抓取姿态对象
    #     g = Grasp()
    #
    #     # 创建夹爪张开、闭合的姿态
    #     g.pre_grasp_posture = self.make_gripper_posture([GRIPPER_OPEN])
    #     g.grasp_posture = self.make_gripper_posture([GRIPPER_CLOSED])
    #
    #     # 设置期望的夹爪靠近、撤离目标的参数
    #     g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [0.0, 0.0, 1.0])
    #     g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])
    #
    #     # 设置抓取姿态
    #     g.grasp_pose = initial_pose_stamped
    #
    #     # 需要尝试改变姿态的数据列表
    #     pitch_vals = [1.51, 1.51, -1.51, 1.512, -1.512, 1.513, -1.513]
    #     yaw_vals = [0]
    #
    #     # 抓取姿态的列表
    #     grasps = []
    #
    #     # 改变姿态，生成抓取动作
    #     for y in yaw_vals:
    #         for p in pitch_vals:
    #             # 欧拉角到四元数的转换
    #             q = quaternion_from_euler(0, p, y)
    #
    #             # 设置抓取的姿态
    #             g.grasp_pose.pose.orientation.x = q[0]
    #             g.grasp_pose.pose.orientation.y = q[1]
    #             g.grasp_pose.pose.orientation.z = q[2]
    #             g.grasp_pose.pose.orientation.w = q[3]
    #
    #             # 设置抓取的唯一id号
    #             g.id = str(len(grasps))
    #
    #             # 设置允许接触的物体
    #             g.allowed_touch_objects = allowed_touch_objects
    #
    #             # 将本次规划的抓取放入抓取列表中
    #             grasps.append(deepcopy(g))
    #
    #     # 返回抓取列表
    #     return grasps
    #
    # # 创建一个允许的放置姿态列表
    # @staticmethod
    # def make_places(init_pose):
    #     # 初始化放置抓取物体的位置
    #     place = PoseStamped()
    #
    #     # 设置放置抓取物体的位置
    #     place = init_pose
    #
    #     # 定义x方向上用于尝试放置物体的偏移参数
    #     x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
    #
    #     # 定义y方向上用于尝试放置物体的偏移参数
    #     y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
    #
    #     pitch_vals = [0]
    #
    #     # 定义用于尝试放置物体的偏航角参数
    #     yaw_vals = [0]
    #
    #     # 定义放置物体的姿态列表
    #     places = []
    #
    #     # 生成每一个角度和偏移方向上的抓取姿态
    #     for y in yaw_vals:
    #         for p in pitch_vals:
    #             for y in y_vals:
    #                 for x in x_vals:
    #                     place.pose.position.x = init_pose.pose.position.x + x
    #                     place.pose.position.y = init_pose.pose.position.y + y
    #
    #                     # 欧拉角到四元数的转换
    #                     q = quaternion_from_euler(0, p, y)
    #
    #                     # 欧拉角到四元数的转换
    #                     place.pose.orientation.x = q[0]
    #                     place.pose.orientation.y = q[1]
    #                     place.pose.orientation.z = q[2]
    #                     place.pose.orientation.w = q[3]
    #
    #                     # 将该放置姿态加入列表
    #                     places.append(deepcopy(place))
    #
    #     # 返回放置物体的姿态列表
    #     return places


class PoseSubscriber:
    def __init__(self):
        self.px = 0
        self.py = 0
        self.pz = 0
        self.ox = 0
        self.oy = 0
        self.oz = 0
        self.w = 0
        rospy.Subscriber('/gazebo/model_states', ModelStates,
                         self.callback, queue_size=1, buff_size=52400)

    def callback(self, data):
        self.px = data.pose[2].position.x
        self.py = data.pose[2].position.y
        self.pz = data.pose[2].position.z
        self.ox = data.pose[2].orientation.x
        self.oy = data.pose[2].orientation.y
        self.oz = data.pose[2].orientation.z
        self.w = data.pose[2].orientation.w
        MoveItPickAndPlace(self.px, self.py, self.pz,
                           self.ox, self.oy, self.oz, self.w)


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('moveit_pick_and_place_demo', anonymous=True)

    # 放置物体
    BoxInGazeboPosePub(0.651988378093, -0.06546923428, 0.489970846917, 1)

    # 接收物体坐标并抓取
    PoseSubscriber()

    rospy.spin()
