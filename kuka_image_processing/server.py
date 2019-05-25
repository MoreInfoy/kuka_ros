import cv2
import os
from keras.models import load_model
import numpy as np
from keras.callbacks import ModelCheckpoint, TensorBoard
from keras.backend.tensorflow_backend  import set_session
import tensorflow as tf
import multiprocessing
import time 
tf_config = tf.ConfigProto()
tf_config.gpu_options.allow_growth = True
set_session(tf.Session(config=tf_config))

# NETWORK = '/home/nesa320/yangshunpeng/ggcnn-master/data/networks/190520_1826__ggcnn_9_5_3__32_16_8/epoch_78_model.hdf5'

NETWORK = '/home/nesa320/yangshunpeng/ggcnn-master/data/190520_1420__ggcnn_9_5_3__32_16_8/epoch_78_model.hdf5'
model = load_model(NETWORK)

def tar():
	os.system('rm sendmsg.xml')
	os.system('tar -xzvf sendmsg.tar.gz')

if __name__ == '__main__':
	loops = 0
	t1 = time.time()
	while loops<100:
	    loops += 1
	    tar()	
	    fs = cv2.FileStorage("sendmsg.xml", cv2.FILE_STORAGE_READ)
	    input_d = fs.getNode('depth_image').mat()
	    input_rgb = fs.getNode('rgb_image').mat()
	    fs.release()

	    inputd_data = np.zeros(shape=(1, 300, 300, 1), dtype=np.float32)
	    inputrgb_data = np.zeros(shape=(1, 300, 300, 3), dtype=np.float32)

	    inputd_data[0, :, :, 0] = input_d[0:300, 0:300]
	    inputrgb_data[0, :, :, :] = input_rgb[0:300, 0:300, :]

	    inputd_data = np.clip((inputd_data - inputd_data.mean()), -1, 1)

	    # prediction
	    model_output_data = model.predict([inputd_data, inputrgb_data])
	    # model_output_data = model.predict(inputd_data)
	    output = np.zeros((1,3), np.float)	    
	    grasp_position_out = model_output_data[0]
            par = np.argmax(grasp_position_out)
	    output[0,0] = par
	    grasp_angles_out = np.arctan2(
		model_output_data[2], model_output_data[3]) / 2
	    output[0,1] = grasp_angles_out[0, par//300, par%300, 0]
	    grasp_width_out = model_output_data[3]
	    output[0,2] = grasp_width_out[0, par//300, par%300, 0]

	    print(output)
	    np.save('result', output)
	    print(input_d.shape)
	    print(input_rgb.shape)
	t2 = time.time()
	print(t2-t1)
