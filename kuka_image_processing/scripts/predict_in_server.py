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
DEPTH_TOPIC = "/kinect/depth/image_raw"
RGB_TOPIC = "/kinect/rgb/image_raw"

NO_GRASPS = 1
ANG_THRESHOLD = 0


def result_show(grasp_position_out, grasp_width_out, grasp_angles_out, depth_image):
    """
    Show the result
    """
    grasp_position_out = gaussian(
        grasp_position_out.squeeze(), 5.0, preserve_range=True)
    grasp_width_out = gaussian(
        grasp_width_out.squeeze(), 1.0, preserve_range=True)
    gs = detect_grasps(grasp_position_out.squeeze(), grasp_angles_out.squeeze(),
                       width_img=grasp_width_out.squeeze(), no_grasps=NO_GRASPS,
                       ang_threshold=ANG_THRESHOLD)
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(1, 1, 1)
    ax.imshow(depth_image)
    for g in gs:
        g.plot(ax)
    plt.show()


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
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        rgb_image = rgb_image[10:310, :, :]
        rgb_image[0:25, :, :] = rgb_image[25, 320, :]
        rgb_image = rgb_image[:, 170:470, :]

        if flag_d_read.value and not flag_rgb_read.value:
            fs = cv2.FileStorage("sendmsg.xml", cv2.FILE_STORAGE_APPEND)
            fs.write("rgb_image", rgb_image)
            fs.release()
            os.system('tar -czf sendmsg.tar.gz sendmsg.xml')
            os.system('/home/nimpng/kuka_ros/src/connect.sh')

            a = np.load('result.npy')
            msg = predict()
            msg.position = a[0,0]
            msg.angle = a[0,1]
            msg.width = a[0,2]
            self.pub.publish(msg)

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

        depth_image = depth_image[10:310, :].copy()
        depth_image[0:24, :] = depth_image[25, 320]
        depth_image = depth_image[:, 170:470]

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
