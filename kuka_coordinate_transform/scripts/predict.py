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

# multipleProcess
input_net = multiprocessing.Array('f', [i for i in range(90000)])
flag = multiprocessing.Value('b', False)
exit_flag = multiprocessing.Value('b', False)
in_processing = multiprocessing.Value('b', False)
NO_GRASPS = 1
ANG_THRESHOLD = 0
INPUT_DATA_FN = '/home/nimpng/kuka_ros/src/kuka_image_processing/data/_val_input.npy'
NETWORK = '/home/nimpng/kuka_ros/src/kuka_image_processing/network/net_model.hdf5'



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


def predict(depth_map):
    print("predict process on")
    model = load_model(NETWORK)
    while not exit_flag.value:
        print("predict process on")
        if flag.value:
            in_processing.value = True
            print("input_net = ", input_net[0:300])
            input_temp = np.array(input_net, dtype=np.float)
            print("input_temp = ", input_temp)
            input_temp = np.reshape(input_temp, (300, 300))

            # format the data for the input of network
            input_data = np.zeros(shape=(1, 300, 300, 1), dtype=np.float32)
            input_data[0, :, :, 0] = input_temp[0:300, 0:300]
            input_data = np.clip((input_data - input_data.mean()), -1, 1)

            # prediction
            model_output_data = model.predict(input_data)

            # extra the data within the result of prediction
            grasp_position_out = model_output_data[0]
            grasp_angles_out = np.arctan2(
                model_output_data[2], model_output_data[3]) / 2
            grasp_width_out = model_output_data[3] * 150
            result_show(grasp_position_out, grasp_width_out,
                        grasp_angles_out, input_temp)
            in_processing.value = False
            flag.value = False
    print("predict process off")
