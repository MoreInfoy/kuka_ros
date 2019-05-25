import cv2
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

label_font = {
    'color' : 'k' ,
    'size' : 15,
    'weight' : 'normal'
}

if __name__ == '__main__':
    fs = cv2.FileStorage('/home/nimpng/network_valid.xml', cv2.FILE_STORAGE_READ)
    X = fs.getNode("advanced_gg_cnn").mat().transpose()
    Y = fs.getNode("original_gg_cnn").mat().transpose()
    plt.plot(range(99), X[:,0], c='r', label='advanced GG-CNN')
    plt.plot(range(99), Y[:,0], c='b', label='original GG-CNN')
    plt.legend(loc="down right") 

    plt.show()
    print(X[98,0])