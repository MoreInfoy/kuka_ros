# Combine ROS-Melodic on Ubuntu 18.04.02 with KUKA KR6 700 SIXX
![image](https://github.com/NimaPng/kuka_ros/blob/master/env.png)

>**Prerequisite knowledge:** \
ROS Programming  , C++ and python

>**Requirements:**\
Opencv 4.0 and ROS Melodic.

>**OpenCV 4.0 installation tutorial:**\
<http://www.codebind.com/linux-tutorials/install-opencv-3-2-ubuntu-18-04-lts-linux/>

+ Build the simulation environment and hardware platform.

+ Use the method of adaptive threshold segmention in computer vision to find the grasping target in a image  taken by Kinect 2.0 with simple backgroud.

+ Test and improve GG-CNN first published by Morrson D, Corke P and  Leitner J in a paper:

    > `Closing the Loop for Robotic Grasping: A Real-time, Generative Grasp Synthesis Approach[J]. arXiv:1804.05172 [cs], 2018`.

+ Use the server to process the neural network and transport the result to local computer, which is used to control robotic arm to grasp the target.

---
---

+ **Packages included**

    `gazebo_grasp_plugin`

    `gazebo_version_helpers`

    The above two files is used to solve the collision problem in gazebo.

    `iai_kinect2`

    The above files is used to connect Kinect 2.0 with local computer, including files about kinect 2.0 calibration. 
    <https://github.com/code-iai/iai_kinect2>

    `image_segment`

    The above files is about how to use the method of computer vision to obtain the coordinate of target in an image.

    `kuka_coordinate_transform`

    The above files is about how to convert the coordinates of target in the camera frame to that in the robot frame.

    `kuka_description`

    `kuka_gazebo`

    `kuka_kr6_support`

    `kuka_resources`

    The above four files is about how to build a KUKA KR6 700 SIXX robotic arm in a simulation environment within rviz and gazebo. More details and materials could be found by the following link: <https://github.com/ros-industrial/kuka_experimental>

    `kuka_image_processing`

    The above files is about the usage of GG-CNN in this situatin.

    `kuka_moveit_config`

    The above files is about how to use MoveIt! in arm's path planning, and the process of configuration could be found in the following website: <https://www.bilibili.com/video/av48016403?from=search&seid=376820350019891062>

    `kuka_eki_hw_interface`

    `kuka_rsi_hw_interface`

    `kuka_rsi_simulator`

    The above three files is about the communication between the KR C4 controller and your own computer. The usage could be found under corresponding folder (**.md).

    `robotiq_85_bringup`

    `robotiq_85_description`

    `robotiq_85_driver`

    `robotiq_85_gripper`

    `robotiq_85_moveit_config`

    `robotiq_85_msg`

    `robotiq_85_simulation`

    The above files is about how to simulate a gripper called robotiq. More details could be found in the following websites: <https://github.com/waypointrobotics/robotiq_85_gripper>

---
---

## Build above packages in ROS workspace

1. Create your own folder "catkin_ws/src" in Ubuntu 18.04.02, run the below command in terminal:

>       mkdir -p catkin_ws/src & cd catkin_ws/src

2. Copy the packages you need to the folder catkin_ws/src

3. Build the packages. Run the following command in terminal

>       cd catkin_ws & catkin_make 

4. Add the following command to the end of .bashrc and run '`source .bashrc`' in terminal:

>       source /your/own/catkin_ws/devel/setup.bash

4. When finshing building, you can use ros command like `roslaunch`,`rosrun` to run the package on your own computer.

---
---

## *The Process of building packages: help you to correct some errors*

    link: https://pan.baidu.com/s/1WLWzZg3ccNwFzP3bupr_Mg
    keys: 1d4e

---
---

## *launch the simulation environment*

    link: https://pan.baidu.com/s/1NYKOJnCEZ4wD1qHstpTycw 
    keys: 52qk

---
---
---
---

# KUKA CONFIG

> Install workvisual

> Read the following operation manual

    link: https://pan.baidu.com/s/10C-aRHcBKkEqFARYVNCZWA
    keys: xeqc

*Note: All config could be done only if you signed in the controller system as an engineer*

---
---

### Connect the controller with your computer

*Note: Change the firewall settings to allow corresponding Ethernet to access any network*


1. Connect your computer and KUKA Robot with network cable

2. Change the IP address of the local Ethernet to be same the network segment as the KUKA Robot. For example, if the IP address of KUKA Robot is 192.168.1.101, then IP address of your computer's Ethernet should be 192.168.1.x (0<x<255 and x != 101)

3. Check the connection, you can use `Ping 192.168.1.101`.

---
---

### How to use workvisual
* Download and install Workvisual 4.x. <https://www.kuka.com/zh-tw/services/downloads?terms=product_name:KUKA.WorkVisual&q=>

* Install KOP in Workviusal. You can copy the KOP file from the D:/the/package/you/used/like/EthernetKRL on the controller KR C4, or download from the following link:
    > link: https://pan.baidu.com/s/1fvfy3Gk5OHXuUdscAd-uxg  \
    keys: uiii

    The following video will show you how to install KOP.
    > link: https://pan.baidu.com/s/1EBUf_UTtueaKWCdYNvJdXg \
    keys: j9s6 
