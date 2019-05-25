# kuka_ros
a platform with ROS, KUKA R6 700 sixx and Kinect 2.0

# environment
* Ubuntu 18.04
* OpenCV 4.0.1
* tensorflow-gpu
* ros-melodic
* libfreenec2

# Run
Download all packages into a dir 'kuka_ros' in '/home/username/', and run the command 'catkin_make' in kuka_ros.

run the following commands in shell:
1. roslaunch kuka_gazebo arm_bringup_moveit.launch
2. rostopic list, see the topic like /Kinect/* and /gazebo/*
3. rosrun kuka_image_processing grasp_run.py (if the grasping failed, change the parameter in the /kuka_description/urdf/gzplugin_grasp_fix.urdf.xacro)
