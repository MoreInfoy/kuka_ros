#!/usr/bin/env python
import numpy as np
from keras.models import load_model
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
import xml.dom.minidom
from kuka_image_processing.msg import predict
# multipleProcess
input_d = multiprocessing.Array('f', [i for i in range(90000)])
input_rgb = multiprocessing.Array('i', [i for i in range(270000)])

flag_rgb_read = multiprocessing.Value('b', True)
flag_d_read = multiprocessing.Value('b', False)

flag = multiprocessing.Value('b', False)
exit_flag = multiprocessing.Value('b', False)
in_processing = multiprocessing.Value('b', False)
output1 = multiprocessing.Value('d', 0.0)
output2 = multiprocessing.Value('d', 0.0)
output3 = multiprocessing.Value('d', 0.0)


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


def _predict_():
    print("predict start")
    model = load_model(NETWORK)
    while not exit_flag.value:
        print("wait for data")
        if flag.value:
            in_processing.value = True
            # print("input_d = ", input_d[0:300])
            inputd_temp = np.array(input_d, dtype=np.float)
            inputrgb_temp = np.array(input_rgb, dtype=np.float)
            # print("input_temp = ", input_temp)
            inputd_temp = np.reshape(inputd_temp, (300, 300))
            inputrgb_temp = np.reshape(inputrgb_temp, (300, 300, 3))
            # plt.imshow(inputrgb_temp)
            # plt.show()
            # format the data for the input of network
            inputd_data = np.zeros(shape=(1, 300, 300, 1), dtype=np.float32)
            inputrgb_data = np.zeros(shape=(1, 300, 300, 3), dtype=np.float32)

            inputd_data[0, :, :, 0] = inputd_temp[0:300, 0:300]
            inputrgb_data[0, :, :, :] = inputrgb_temp[0:300, 0:300, :]

            inputd_data = np.clip((inputd_data - inputd_data.mean()), -1, 1)

            # prediction
            model_output_data = model.predict([inputd_data, inputrgb_data])

            # extra the data within the result of prediction
            # grasp_position_out = model_output_data[0]
            # grasp_angles_out = np.arctan2(
            #     model_output_data[2], model_output_data[3]) / 2
            # grasp_width_out = model_output_data[3] * 150
            # result_show(grasp_position_out, grasp_width_out,
            #             grasp_angles_out, inputd_temp)

            # output = np.zeros((1,3), np.float)	    
            grasp_position_out = model_output_data[0]
            par = np.argmax(grasp_position_out)
            # output[0,0] = par
            output1.value = par
            grasp_angles_out = np.arctan2(
            model_output_data[2], model_output_data[3]) / 2
            # output[0,1] = grasp_angles_out[0, par//300, par%300, 0]
            output2.value = grasp_angles_out[0, par//300, par%300, 0]
            grasp_width_out = model_output_data[3]
            # output[0,2] = grasp_width_out[0, par//300, par%300, 0]
            output3.value = grasp_width_out[0, par//300, par%300, 0]
            # print(output)

            # np.save('result', output)
            in_processing.value = False
            flag.value = False
    print("predict process off")


class ImageConverter:

    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
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

        if not in_processing.value and flag_d_read.value and not flag_rgb_read.value:
            _par_ = np.reshape(rgb_image, (1, 270000))[0:270000]
            for i in range(270000):
                input_rgb[i] = _par_[0, i]
            # print(input_d)
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

        if not in_processing.value and flag_rgb_read.value and not flag_d_read.value:
            _par_ = np.reshape(depth_image, (1, 90000))[0:90000]
            for i in range(90000):
                input_d[i] = _par_[0, i]
            # print(input_d)
            flag.value = True
            flag_rgb_read.value = False
            flag_d_read.value = True
            print("get depth image")
            msg = predict()
            msg.position = output1.value
            msg.angle = output2.value
            msg.width = output3.value
            self.pub.publish(msg)
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


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ImageConverter()
    p = multiprocessing.Process(target=_predict_)
    p.run()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
