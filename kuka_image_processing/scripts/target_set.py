#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
import numpy as np

BOX_ID = 'box'
BOX_SIZE = [0.05, 0.092474, 0.05]
MODEL_NAME = 'unit_box_0'
REFERENCE_FRAME = 'base_link'


class BoxInGazeboPosePub:
    def __init__(self, px, py, pz, w):
        rospy.init_node('BoxInGazeboPosePublisher', anonymous=True)
        self.pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
        self.scene = PlanningSceneInterface()

        self.pose_msg = ModelState()
        self.pose_msg.model_name = MODEL_NAME
        self.pose_msg.reference_frame = REFERENCE_FRAME
        self.pose_msg.pose.position.x = px
        self.pose_msg.pose.position.y = py
        self.pose_msg.pose.position.z = pz
        self.pose_msg.pose.orientation.w = w

        self.box_pose = PoseStamped()
        self.box_pose.header.frame_id = REFERENCE_FRAME
        self.box_pose.pose.position.x = px
        self.box_pose.pose.position.y = py
        self.box_pose.pose.position.z = pz
        self.box_pose.pose.orientation.w = w

        self.publisher()

    def publisher(self):
        rate = rospy.Rate(10)
        theta = 0
        while not rospy.is_shutdown():
            x = 0.64988378093 + 0.07 * np.cos(theta)
            y = 0.1 * np.sin(theta)
            self.pose_msg.pose.position.x = x
            self.pose_msg.pose.position.y = y
            self.box_pose.pose.position.x = x
            self.box_pose.pose.position.y = y
            self.pub.publish(self.pose_msg)
            self.scene.remove_world_object(BOX_ID)
            self.scene.add_box(BOX_ID, self.box_pose, BOX_SIZE)
            theta += np.pi / 30
            rate.sleep()
            # theta += 1
            # print(theta)
            # if theta > 1:
                # break


if __name__ == '__main__':

    try:
        BoxInGazeboPosePub(px=.811988378093, py=-0.376546923428, pz=0.489970846917, w=1.0)
    except rospy.ROSInterruptException:
        pass
