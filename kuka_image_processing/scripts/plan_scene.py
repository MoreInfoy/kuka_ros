#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np
import moveit_commander
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

BOX_ID = 'box'
BOX_SIZE = [0.05, 0.092474, 0.05]
COLORS = dict()
SCENE = PlanningSceneInterface()


class BoxInPlanScene:
    def __init__(self):
        rospy.init_node('plan_scene', anonymous=True)
        # self.scene = PlanningSceneInterface()

        # self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)

    def callback(self, data):
        # print('callback')
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'base_link'
        box_pose.pose.position.x = data.pose[2].position.x
        box_pose.pose.position.y = data.pose[2].position.y
        box_pose.pose.position.z = data.pose[2].position.z
        # box_pose.pose.orientation.x = data.pose[2].orientation.x
        # box_pose.pose.orientation.y = data.pose[2].orientation.y
        # box_pose.pose.orientation.y = data.pose[2].orientation.z
        box_pose.pose.orientation.w = data.pose[2].orientation.w

        SCENE.remove_world_object(BOX_ID)
        # rospy.sleep(1)
        SCENE.add_box(BOX_ID, box_pose, BOX_SIZE)
        # rospy.sleep(2)
        # self.set_color(BOX_ID, 0.8, 0.4, 0, 1.0)
        # self.send_colors()
        # rospy.sleep(1.0)

    # @staticmethod
    # def set_color(name, r, g, b, a=0.9):
    #     # 初始化moveit颜色对象
    #     color = ObjectColor()
    #
    #     # 设置颜色值
    #     color.id = name
    #     color.color.r = r
    #     color.color.g = g
    #     color.color.b = b
    #     color.color.a = a
    #
    #     # 更新颜色字典
    #     COLORS[name] = color
    #
    # def send_colors(self):
    #     # 初始化规划场景对象
    #     p = PlanningScene()
    #
    #     # 需要设置规划场景是否有差异
    #     p.is_diff = True
    #
    #     # 从颜色字典中取出颜色设置
    #     for color in COLORS.values():
    #         p.object_colors.append(color)
    #
    #     # 发布场景物体颜色设置
    #     self.scene_pub.publish(p)


if __name__ == '__main__':

    try:
        BoxInPlanScene()
    except rospy.ROSInterruptException:
        pass
