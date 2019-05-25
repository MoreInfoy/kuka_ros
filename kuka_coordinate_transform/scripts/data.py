#!ecoding=utf-8
import cv2
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

label_font = {
    'color': 'k',
    'size': 15,
    'weight': 'normal'
}

if __name__ == '__main__':
    fs = cv2.FileStorage(
        '/home/nimpng/cal_error.xml', cv2.FILE_STORAGE_READ)
    Y = fs.getNode("Y").mat().transpose()

    # plt.plot(range(78),X[0:78, 0], 'c', label='Improved GG-CNN')
    plt.plot(np.arange(0,50,5),Y, 'm', label='GG-CNN')
    plt.xlabel('samples')
    plt.ylabel('MSE')
    # plt.legend(prop={'family' : 'Times New Roman', 'size'   : 12})
    plt.title('calibration errors')
    # fs_bad = cv2.FileStorage(
    #     '/home/nimpng/pos_transform_matrix.xml', cv2.FILE_STORAGE_READ)
    # X = fs.getNode("X").mat().transpose()
    # Y = fs.getNode("Y").mat().transpose()

    # X_bad = fs_bad.getNode("X").mat().transpose()
    # plt.scatter(X[:, 0], X[:, 1], c='b', s=80,
    #             marker='o', label='positive point')
    # plt.scatter(X_bad[:, 0], X_bad[:, 1], c='c',
    #             s=80, marker='x', label='bad point')
    # plt.title("Vertical View - Spatial sampling point distribution",
    #           alpha=0.6, color="k", size=15, weight='normal', backgroundcolor="w")
    # plt.legend(loc="upper left", fontsize=15)  # legend的位置位于左上
    # plt.xlabel("X axis", fontdict=label_font)
    # plt.ylabel("Y axis", fontdict=label_font)
#     # x,z = np.meshgrid(X_bad[:,0], X_bad[:,2])
#     # y = x + 170
#     # # z = np.linspace(100, 1000, 2)
#     # fig = plt.figure(facecolor='blue',edgecolor='black')
#     # ax = Axes3D(fig)
#     # ax.grid(False)
#     # ax.set_axis_on()
#     # ax.scatter(X[:,0], X[:,1], X[:,2], c='b', s=80, marker='o', label='positive point')
#     # ax.scatter(X_bad[:,0], X_bad[:,1], X_bad[:,2], c='c', s=80, marker='x', label='bad point')
#     # # ax.plot_surface(x,y,z,rstride=1, cstride=1, cmap=None)
#     # ax.set_xlabel("X axis", fontdict=label_font)
#     # ax.set_ylabel("Y axis", fontdict=label_font)
#     # ax.set_zlabel("Z axis", fontdict=label_font)
#     # ax.set_title("Spatial sampling point distribution", alpha=0.6, color="k", size=15, weight='normal', backgroundcolor="w")
#     # ax.legend(loc="upper left")   #legend的位置位于左上
    plt.show()
# #     print(X.shape)
# #     plt.scatter()
# #     print(Y.mat().shape)

# # import numpy as np
# # import matplotlib as mpl
# # import matplotlib.pyplot as plt
# # from mpl_toolkits.mplot3d import Axes3D

# # label_font = {
# #     'color' : 'c' ,
# #     'size' : 15,
# #     'weight' : 'bold'
# # }

# # def randrange(n, vmin, vmax):
# #     r = np.random.rand(n)   #随机生成n 个介于0-1之间的数
# #     return (vmax-vmin) * r + vmin   #得到n个[vmin，vmax]之间的随机数


# # fig = plt.figure(figsize=(16,12))   # 参数依然是图片大小
# # ax = fig.add_subplot(111, projection='3d')      #确定子坐标轴，111表示1行1列的第一个图   要同时画好几个图的时候可以用这个

# # #准备数据
# # n = 200
# # for zlow, zhigh, c, m, l in [(4, 15, 'r', 'o', 'posirtive'), (13, 40, 'g', '*', 'negative')]:  #用两个tuple（画笔）,是为了将形状和颜色区别开来
# #     x = randrange(n, 15, 40)
# #     y = randrange(n, -5, 25)
# #     z = randrange(n, zlow, zhigh)
# #     ax.scatter(x, y, z, c=c, marker=m, label=l, s=z * 10)   # marker的尺寸和z的大小成正比

# # ax.set_xlabel("X axis", fontdict=label_font)
# # ax.set_ylabel("Y axis", fontdict=label_font)
# # ax.set_zlabel("Z axis", fontdict=label_font)
# # ax.set_title("Scatter plot", alpha=0.6, color="b", size=25, weight='bold', backgroundcolor="y")   #子图（其实就一个图）的title
# # ax.legend(loc="upper left")   #legend的位置位于左上


# # plt.show()


# import matplotlib.pyplot as plt

# name_list = ['Monday', 'Tuesday', 'Friday', 'Sunday']
# s1 = [1.2476e+06, 1.26713e+06]
# s2 = [103400, 909205]
# s3 = [0.00342384, 540041]
# x = list(range(len(s1)))
# y = list(range(len(s1)))
# total_width, n = 0.8, 3
# width = total_width / n

# b1 = plt.bar(x, s1, width=width, label='Singular v1', fc='b', )
# for i in range(len(x)):
#     x[i] = x[i] + width
#     y[i] = x[i] + width
# b2 = plt.bar(x, s2, width=width, label='Singular v2',  fc='m', tick_label=['Bad Points SVD','Positive Points SVD' ])
# b3 = plt.bar(y, s3, width=width, label='Singular v3',  fc='c')
# for b in b1+b2+b3:
#     h = b.get_height()
#     plt.text(b.get_x()+b.get_width()/2, h, '%.3e'%float(h), ha='center', va='bottom',size=10)
# plt.legend()
# plt.title("SVD of matrix w obtained from sampling points",
#               alpha=0.6, color="k", size=15, weight='normal', backgroundcolor="w")
# plt.show()
