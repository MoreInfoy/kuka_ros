#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from image_segment.msg import target
import multiprocessing
from image_segment.msg import target

DEPTH_TOPIC = "/kinect2/depth"
RGB_TOPIC = "/kinect2/rgb"
DEPTH_WIN = "DEPTH WIN"
RGB_WIN = "RGB_WIN"
CONTOUR_WIN = "CONTOUR_WIN"

camera_cx = 2.5738909838479350e+02
camera_cy = 2.0617431757438302e+02
fx = 3.6095753862475351e+02
fy = 3.6068889959341760e+02


rect_roi = multiprocessing.Array("f", [i for i in range(4)])
in_pub = multiprocessing.Value('b', False)
get_pos = multiprocessing.Value('b', False)
exit_flag = multiprocessing.Value('b', False)


def pub_tar_pos():
    pub = rospy.Publisher(
        "/target_position", target, queue_size=1)
    tg = target()
    print("pub is ready")
    while not exit_flag.value:
        print("waiting")
        if get_pos.value:
            print("publishing")
            in_pub.value = True
            tg.x = (rect_roi[0] - camera_cx) * rect_roi[2] / fx
            tg.y = (rect_roi[1] - camera_cy) * rect_roi[2] / fy
            tg.z = rect_roi[2]
            tg.angle = rect_roi[3]
            pub.publish(tg)
            in_pub.value = False
            get_pos.value = False


def rgb_callback(imgmsg):
    rgb_image = None
    try:
        rgb_image = CvBridge().imgmsg_to_cv2(
            img_msg=imgmsg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.loginfo(e)
    
    roi_image = rgb_image[100:200, 135:335,:].copy()
    roirgb_image = cv2.cvtColor(roi_image, cv2.COLOR_BGRA2BGR)
    cv2.imwrite("rgb.png", roirgb_image)
    roi_image = cv2.cvtColor(roirgb_image, cv2.COLOR_BGR2GRAY)

    roi_threshold = cv2.adaptiveThreshold(
        roi_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 7)

    # up sample
    roi_threshold = cv2.pyrUp(roi_threshold)

    # median filter
    cv2.medianBlur(roi_threshold, 3, roi_threshold)

    # find the contours
    contours, hierarchy = cv2.findContours(
        roi_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    roi_image = cv2.pyrUp(roi_image)
    roi_image = cv2.pyrUp(roirgb_image)

    # roi_image = cv2.cvtColor(roi_image, cv2.COLOR_GRAY2BGR)

    # find the rectange or minArea rectangle
    first = True
    for c in contours:

        x, y, w, h = cv2.boundingRect(c)
        if w < 10 and h < 10:
            continue
        cv2.rectangle(roi_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        if first and not in_pub.value:
            x_rect = int(np.mean(box, axis=0)[0] / 2.0) + 135
            y_rect = int(np.mean(box, axis=0)[1] / 2.0) + 100
            rect_roi[0] = float(x_rect)
            rect_roi[1] = float(y_rect)
            rect_roi[2] = rgb_image[y_rect, x_rect, 0]

        box = np.int0(box)

        if first and not in_pub.value:
            first = False
            p_d = box[np.argmin(box, axis=0)[1], :]  # smallest y
            p_u = box[(np.argmin(box, axis=0)[1] + 2) % 4, :]
            p_l = box[(np.argmin(box, axis=0)[1] + 1) % 4, :]
            p_r = box[(np.argmin(box, axis=0)[1] + 3) % 4, :]

            if p_l[0] > p_r[0]:
                par = p_l.copy()
                p_l = p_r.copy()
                p_r = par
            # print(float(p_r[1]-p_d[1])/float(p_r[0]-p_d[0]))
            rect_roi[3] = np.arctan(float(p_r[1]-p_d[1])/float(p_r[0]-p_d[0])) * 180.0 / np.pi
            get_pos.value = True

        cv2.drawContours(roi_image, [box], 0, (0, 0, 255), 3)
        cv2.drawContours(roirgb_image, [box], 0, (0, 0, 255), 3)


    cv2.namedWindow(CONTOUR_WIN, cv2.WINDOW_NORMAL)
    cv2.imshow(CONTOUR_WIN, roi_image)
    cv2.imwrite("segment_rgb.png", roi_image)
    # cv2.imwrite("segment_roirgb.png", roirgb_image)

    cv2.imwrite("threshold_rgb.png", roi_threshold)

    cv2.rectangle(rgb_image, (135, 100),
                  (135+200, 100+100), (0, 255, 0), 2, 8)

    cv2.rectangle(rgb_image, (x_rect-10, y_rect-10),
                  (x_rect+10, y_rect+10), (0, 255, 0), 2, 8)

    cv2.imshow(RGB_WIN, rgb_image)
    cv2.waitKey(3)
    # cv2.namedWindow(DEPTH_WIN, cv2.WINDOW_AUTOSIZE)
    # cv2.namedWindow(RGB_WIN, cv2.WINDOW_AUTOSIZE)
    # cv2.startWindowThread()


def depth_callback(imgmsg):
    depth_image = None
    try:
        depth_image = CvBridge().imgmsg_to_cv2(
            img_msg=imgmsg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.loginfo(e)

    depth_data = depth_image.copy()
    roi_image = depth_image[100:200, 135:335].copy()
    need_image = cv2.imread('rgb.png', 1)
    # roi_image[roi_image > 1245.0] = 1255.0
    _par_ = float(roi_image[99, 99].copy())
    # roi_image = cv2.copyMakeBorder(roi_image, 0, 50, 0, 50,
                                #    cv2.BORDER_CONSTANT, value=[_par_, _par_, _par_])

    # # pass the value to the prediction process, only use in multiple process
    # if not in_processing.value:
    #     _par_ = np.reshape(cv2.pyrUp(roi_image), (1, 90000))[0:90000]
    #     for i in range(90000):
    #         input_net[i] = _par_[0, i]
    #     # print(input_net)
    #     flag.value = True

    # binary image
    roi_image = cv2.convertScaleAbs(roi_image, alpha=0.8, beta=-800)

    roi_threshold = cv2.adaptiveThreshold(
        roi_image, 1260.0, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 7)

    # up sample
    roi_threshold = cv2.pyrUp(roi_threshold)

    # median filter
    cv2.medianBlur(roi_threshold, 3, roi_threshold)

    # find the contours
    contours, hierarchy = cv2.findContours(
        roi_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    roi_image = cv2.pyrUp(roi_image)
    need_image = cv2.pyrUp(need_image)

    roi_image = cv2.cvtColor(roi_image, cv2.COLOR_GRAY2BGR)

    # find the rectange or minArea rectangle
    first = True
    for c in contours:

        x, y, w, h = cv2.boundingRect(c)
        if w < 20 or h < 20:
            continue
        cv2.rectangle(roi_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.rectangle(need_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        if first and not in_pub.value:
            x_rect = int(np.mean(box, axis=0)[0] / 2.0) + 135
            y_rect = int(np.mean(box, axis=0)[1] / 2.0) + 100
            rect_roi[0] = float(x_rect)
            rect_roi[1] = float(y_rect)
            rect_roi[2] = depth_data[y_rect, x_rect]

        box = np.int0(box)

        if first and not in_pub.value:
            first = False
            p_d = box[np.argmin(box, axis=0)[1], :]  # smallest y
            p_u = box[(np.argmin(box, axis=0)[1] + 2) % 4, :]
            p_l = box[(np.argmin(box, axis=0)[1] + 1) % 4, :]
            p_r = box[(np.argmin(box, axis=0)[1] + 3) % 4, :]

            if p_l[0] > p_r[0]:
                par = p_l.copy()
                p_l = p_r.copy()
                p_r = par
            # print(float(p_r[1]-p_d[1])/float(p_r[0]-p_d[0]))
            rect_roi[3] = np.arctan(float(p_r[1]-p_d[1])/float(p_r[0]-p_d[0])) * 180.0 / np.pi
            get_pos.value = True

        cv2.drawContours(roi_image, [box], 0, (0, 0, 255), 3)
        cv2.drawContours(need_image, [box], 0, (0, 0, 255), 3)


    cv2.namedWindow(CONTOUR_WIN, cv2.WINDOW_NORMAL)
    cv2.imshow(CONTOUR_WIN, roi_image)
    cv2.imwrite("segment_depth.png", roi_image)
    cv2.imwrite("segment_rgb_depth.png", need_image)

    cv2.imwrite("threshold_depth.png", roi_threshold)

    cv2.rectangle(depth_image, (135, 100),
                  (135+200, 100+100), (0, 255, 0), 2, 8)

    cv2.rectangle(depth_image, (x_rect-10, y_rect-10),
                  (x_rect+10, y_rect+10), (0, 255, 0), 2, 8)

    
    cv2.namedWindow(DEPTH_WIN, cv2.WINDOW_NORMAL)
    cv2.imshow(DEPTH_WIN, depth_image / 4500.0)

    # stop all process and exit
    if cv2.waitKey(30) == 27:
        # print("exit")
        exit_flag.value = True  # only use in multiple process
        rospy.signal_shutdown("Esc")


if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    pub = rospy.Publisher("/target_position", target, queue_size=1)
    # rgb_sub = rospy.Subscriber(RGB_TOPIC, Image, rgb_callback)
    depth_sub = rospy.Subscriber(DEPTH_TOPIC, Image, depth_callback)

    p = multiprocessing.Process(target=pub_tar_pos)
    p.run()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
