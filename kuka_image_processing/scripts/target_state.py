#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates


class PoseSubscriber:
    def __init__(self):
        self.px = 0
        self.py = 0
        self.pz = 0
        self.ox = 0
        self.oy = 0
        self.oz = 0
        self.w = 0
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)

    def callback(self, data):
        self.px = data.pose[2].position.x
        self.py = data.pose[2].position.y
        self.pz = data.pose[2].position.z
        self.ox = data.pose[2].orientation.x
        self.oy = data.pose[2].orientation.y
        self.oz = data.pose[2].orientation.z

if __name__ == '__main__':
    rospy.init_node('pose_subscriber', anonymous=True)
    pose = PoseSubscriber()
    rospy.spin()
