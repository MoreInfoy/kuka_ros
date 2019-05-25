#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_segment/target.h>

tf::Vector3 _target_pos, _target_in_world;
double angle;

int callback(const image_segment::targetConstPtr &msg)
{
  _target_pos.setX(msg->x);
  _target_pos.setY(msg->y);
  _target_pos.setZ(msg->z);
  std::cout << _target_pos.x() << ", " << _target_pos.y() << ", " << _target_pos.z() << std::endl;
}

int angle_callback(const image_segment::targetConstPtr &msg)
{
  angle = msg->angle;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle nh;

  tf::TransformListener listener;

  ros::Subscriber sub_target = nh.subscribe<image_segment::target>("/target_position", 1, callback);
  ros::Subscriber sub_angle = nh.subscribe<image_segment::target>("/predict_angle", 1, angle_callback);

  ros::Publisher pub = nh.advertise<image_segment::target>("/target_world_coodinate", 1);

  ros::Rate rate(1);
  while (nh.ok())
  {
    tf::StampedTransform transform1, transform2;
    try
    {
      listener.lookupTransform("/base_link", "/kinect_link",
                               ros::Time(0), transform1);
      listener.lookupTransform("/kinect_link", "/kinect_frame_optical", ros::Time(0), transform2);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    tf::Matrix3x3 _rotation_kinect2word_ = transform1.getBasis();
    tf::Vector3 _origin_kinect2word_ = transform1.getOrigin();
    tf::Matrix3x3 _rotation_camera2kinect_ = transform2.getBasis();
    tf::Vector3 _origin_camera2kinect_ = transform2.getOrigin();
    _target_in_world = _rotation_kinect2word_ * (_rotation_camera2kinect_ * _target_pos + _origin_camera2kinect_) + _origin_kinect2word_;

    image_segment::target msg_tf;
    msg_tf.x = _target_in_world.x();
    msg_tf.y = _target_in_world.y();
    msg_tf.z = _target_in_world.z();
    msg_tf.angle = angle;

    ROS_INFO("get coordinate: %f, %f, %f\n", _target_in_world.x(), _target_in_world.y(), _target_in_world.z());

    pub.publish(msg_tf);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}