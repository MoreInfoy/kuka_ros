#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from skimage.filters import gaussian
from matplotlib import pyplot as plt
import numpy as np
# from keras.models import load_model
from grasp import BoundingBoxes, detect_grasps

NETWORK = '/home/nimpng/kuka_ros/src/kuka_image_processing/network/net_model.hdf5'
INPUT_DATA_FN = '/home/nimpng/kuka_ros/src/kuka_image_processing/data/_val_input.npy'
DEPTH_TOPIC = "/kinect/depth/image_raw"
NO_GRASPS = 1
ANG_THRESHOLD = 0


class ImageConverter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)

        self.bridge = CvBridge()
        self.image_depth_sub = rospy.Subscriber(
            DEPTH_TOPIC, Image, self.depth_callback)

        self._depth_image = None
        self.grasp_position_out = None
        self.grasp_angles_out = None
        self.grasp_width_out = None

    def rgb_callback(self, data):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        rgb_image = rgb_image[115:320, 135:505, :]

        cv2.imshow("Image window", rgb_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(rgb_image, "passthrough"))
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        """
        Callback when receive the message from TOPIC '/kinect/depth/image_raw'
        :param data: the depth image
        :return: None
        """
        model = load_model(NETWORK)
        try:
            self._depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        depth_image = self._depth_image

        # format the data for the input of network
        # input_data = np.zeros(shape=(1, 300, 300, 1), dtype=np.float32)
        # input_data[0, :, :, 0] = self._depth_image[390:690, 782:1082]
        # input_data = np.clip((input_data - input_data.mean()), -1, 1)

        # prediction
        # model_output_data = model.predict(input_data)

        # extra the data within the result of prediction
        # self.grasp_position_out = model_output_data[0]
        # self.grasp_angles_out = np.arctan2(
        #     model_output_data[2], model_output_data[3]) / 2
        # self.grasp_width_out = model_output_data[3] * 150
        # self.result_show()

        # plot a rectangle to indicate the deepth image taken for prediction
        # cv2.normalize(depth_image, depth_image, 100, 255, cv2.NORM_MINMAX)
        # depth_image = depth_image.astype(np.uint8)
        # cv2.rectangle(depth_image, (720, 320), (1020, 620), (0, 255, 0), 2)

        # show the image taken by the kinect
        cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
        cv2.imshow("Depth window", depth_image / 3.0)
        cv2.waitKey(3)

        # publish the image
        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(depth_image, "passthrough"))
        except CvBridgeError as e:
            print(e)

    def result_show(self):
        """
        Show the result
        """
        self.grasp_position_out = gaussian(
            self.grasp_position_out.squeeze(), 5.0, preserve_range=True)
        self.grasp_width_out = gaussian(
            self.grasp_width_out.squeeze(), 1.0, preserve_range=True)
        gs = detect_grasps(self.grasp_position_out.squeeze(), self.grasp_angles_out.squeeze(),
                           width_img=self.grasp_width_out.squeeze(), no_grasps=NO_GRASPS,
                           ang_threshold=ANG_THRESHOLD)
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(1, 1, 1)
        ax.imshow(self._depth_image[390:690, 362:662])
        for g in gs:
            g.plot(ax)
        plt.show()


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ImageConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
