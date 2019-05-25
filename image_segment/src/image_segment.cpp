#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>

#include <image_segment/target.h>

using namespace std;
using namespace cv;

#define RGB_W "RGB_Image_Viewer"
#define DEPTH_W "DEPTH_IMAGE_Viewer"
// #define SUBSCRIBE_TOPIC_RGB "/kinect2/rgb"
// #define SUBSCRIBE_TOPIC_DEPTH "/kinect2/depth"

/* simulate */
#define SUBSCRIBE_TOPIC_RGB "/kinect/rgb/image_raw"
#define SUBSCRIBE_TOPIC_DEPTH "/kinect/depth/image_raw"


const double camera_factor = 1;
const double camera_cx = 2.5738909838479350e+02;
const double camera_cy = 2.0617431757438302e+02;
const double fx = 3.6095753862475351e+02;
const double fy = 3.6068889959341760e+02;

// const double camera_cx = 960.5;
// const double camera_cy = 540.5;
// const double fx = 1662.764073573561;
// const double fy = 1662.764073573561;

Rect rect(140, 135, 200, 100);
int blockSize = 7;
int constValue = 5;
int flag = 0;
/* target postion*/
int pixel_x = 0, pixel_y = 0;
float x, y, z;
bool _rgb_readed = false;

ros::Publisher target_pub;

bool compute_coordinate(int &m, int &n, float &depth_value)
{
    depth_value /= camera_factor;
    x = (m - camera_cx) * depth_value / fx;
    y = (n - camera_cy) * depth_value / fy;
    z = depth_value;

    image_segment::target tg;
    tg.x = x;
    tg.y = y;
    tg.z = z;

    target_pub.publish(tg);
    return true;
}

int depth_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_INFO("in depth_callback");
    try
    {
        Mat depth = cv_bridge::toCvShare(msg, msg->encoding)->image;
        if (depth.empty())
            return 0;
        // cout << depth(rect) << endl;
        // flip(depth, depth, 1);

        // if (!compute_coordinate(pixel_x, pixel_y, depth.at<float>(pixel_y, pixel_x)))
        // {
        //     std::cerr << "compute_coordinate failed" << std::endl;
        //     return -1;
        // }
        // Rect rect_sign(pixel_x - 5, pixel_y - 5, 10, 10);
        // rectangle(depth, rect_sign, Scalar(0, 0, 255), 2, 8);
        rectangle(depth, rect, Scalar(0, 0, 255), 2, 8);
        // string pos = "(" + to_string(x) + ", " + to_string(y) + ", " + to_string(z) + ")";
        // putText(depth, pos, Point2i(pixel_x - 200, pixel_y - 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
        imshow(DEPTH_W, depth / 3.0);
        int key = cv::waitKey(3);
        if (key == 's')
        {
            flag++;
            FileStorage fs("kinect2World.xml", FileStorage::APPEND);
            fs << ("depth" + to_string(flag)) << depth;
            fs.release();
            ROS_INFO("depth %d data saved\n", flag);
        }

        // ROS_INFO("coordinate from depth_image: (%f, %f, %f)\n", x, y, z);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("this %s", e.what());
        return 0;
    }
    return 0;
}

int rgb_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_INFO("in rgb_callback");
    try
    {
        Mat rgb = cv_bridge::toCvShare(msg, msg->encoding)->image(rect);
        // ROS_INFO("%s", msg->encoding.c_str());
        if (rgb.empty())
            return -1;
        // Mat bgr;
        // if (rgb.channels() == 3)
        //     cvtColor(rgb, bgr, COLOR_RGB2BGR);
        // else
        //     return -1;

        // Mat gray, _threshold;
        // cvtColor(rgb, gray, COLOR_BGR2GRAY);
        // cv::adaptiveThreshold(gray, _threshold, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
        // imshow("Gray", gray);
        // imshow("Threshold", _threshold);
        imshow(RGB_W, rgb);
        int key = waitKey(3);
        if (key == 's')
        {
            FileStorage fs("kinect2World.xml", FileStorage::APPEND);
            fs << ("rgb" + to_string(flag)) << rgb;
            fs.release();
            ROS_INFO("rgb %d data saved\n", flag);
        }

        // get the center of the segment
        // int cols = _threshold.cols;
        // int rows = _threshold.rows;
        // int count = 0, _cols_sum = 0, _rows_sum = 0, pixel = 0;
        // for (int i = 0; i < rows; ++i)
        //     for (int j = 0; j < cols; ++j)
        //     {
        //         pixel = _threshold.at<uchar>(i, j);
        //         if (pixel == 255)
        //         {
        //             count++;
        //             _cols_sum += j;
        //             _rows_sum += i;
        //         }
        //     }
        // pixel_y = _rows_sum / count + 135;
        // pixel_x = _cols_sum / count + 140;
        // _rgb_readed = true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("this %s", e.what());
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_segment");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    target_pub = nh.advertise<image_segment::target>("/target_position", 1);

    namedWindow(RGB_W, WINDOW_NORMAL);
    namedWindow(DEPTH_W, WINDOW_AUTOSIZE);
    // namedWindow("threshold", WINDOW_AUTOSIZE);
    // namedWindow("grayImage", WINDOW_AUTOSIZE);
    startWindowThread();

    image_transport::Subscriber sub_rgb = it.subscribe(SUBSCRIBE_TOPIC_RGB, 1, rgb_callback);
    image_transport::Subscriber sub_depth = it.subscribe(SUBSCRIBE_TOPIC_DEPTH, 1, depth_callback);
    ros::spin();

    destroyAllWindows();
    return 0;
}
