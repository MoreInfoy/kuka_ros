#!/usr/bin/env python
import numpy as np
import requests
# from keras.models import load_model
from grasp import BoundingBoxes, detect_grasps
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from skimage.filters import gaussian
from matplotlib import pyplot as plt
import multiprocessing
import rospy
import sys
import socket
import xml.dom.minidom as minidom
import os
from kuka_image_processing.msg import predict

# multipleProcess
flag_rgb_read = multiprocessing.Value('b', True)
flag_d_read = multiprocessing.Value('b', False)
flag_send = multiprocessing.Value('b', False)

NO_GRASPS = 1
ANG_THRESHOLD = 0
NETWORK = '/home/nimpng/kuka_ros/src/kuka_image_processing/network/net_advanced.hdf5'
DEPTH_TOPIC = "/kinect2/depth"
RGB_TOPIC = "/kinect2/rgb"

NO_GRASPS = 1
ANG_THRESHOLD = 0


class ImageConverter:

    def __init__(self):
        self.pub = rospy.Publisher("predict", predict, queue_size=1)

        self.bridge = CvBridge()
        self.image_depth_sub = rospy.Subscriber(
            DEPTH_TOPIC, Image, self.depth_callback)

        self.image_rgb_sub = rospy.Subscriber(
            RGB_TOPIC, Image, self.rgb_callback)

    def rgb_callback(self, data):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

        # cv2.cvtColor(rgb_image, rgb_image, cv2.COLOR_BGRA2BGR)
        roi_image = rgb_image[100:200, 135:235,:].copy()
        roirgb_image = cv2.cvtColor(roi_image, cv2.COLOR_BGRA2BGR)
        roirgb_image = cv2.resize(roirgb_image, (300, 300), interpolation=cv2.INTER_CUBIC)
        cv2.rectangle(rgb_image, (135, 100), (235, 200), (0,0,255), 1, 8)
        # cv2.imshow("rgb", rgb_image)
        # cv2.imshow("roi", roirgb_image)

        # cv2.waitKey(3)
        # rgb_image = rgb_image[10:310, :, :]
        # rgb_image[0:25, :, :] = rgb_image[27, 320, :]
        # rgb_image = rgb_image[:, 170:470, :]

        if flag_d_read.value and not flag_rgb_read.value:
            fs = cv2.FileStorage("sendmsg.xml", cv2.FILE_STORAGE_APPEND)
            fs.write("rgb_image", roirgb_image)
            fs.release()
            os.system('tar -czf sendmsg.tar.gz sendmsg.xml')
            os.system('/home/nimpng/kuka_ros/src/connect.sh')

            a = np.load('result.npy')
            msg = predict()
            msg.position = a[0,0]
            msg.angle = a[0,1]
            msg.width = a[0,2]
            self.pub.publish(msg)
            rospy.sleep(3)
            flag_rgb_read.value = True
            flag_d_read.value = False
            print("get rgb image")

    def depth_callback(self, data):
        """
        Callback when receive the message from TOPIC '/kinect/depth/image_raw'
        :param data: the depth image
        :return: None
        """
        # print("in depth")
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

        depth_image = depth_image[100:200, 135:235].copy()
        depth_image = cv2.resize(depth_image, (300, 300), interpolation=cv2.INTER_CUBIC)

        if flag_rgb_read.value and not flag_d_read.value:
            fs = cv2.FileStorage("sendmsg.xml", cv2.FILE_STORAGE_WRITE)
            fs.write("depth_image", depth_image)
            fs.release()
            flag_rgb_read.value = False
            flag_d_read.value = True
            print("get depth image")


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    # if connect_server():
    main(sys.argv)
    # grasp_position_out = model_output_data[0]
    # grasp_angles_out = np.arctan2(
    #     model_output_data[2], model_output_data[3]) / 2
    # grasp_width_out = model_output_data[3] * 150
    # result_show(grasp_position_out, grasp_width_out,
    #             grasp_angles_out, inputd_temp)
