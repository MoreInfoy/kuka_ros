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
#include <kuka_image_processing/predict.h>

using namespace std;
using namespace cv;

#define RGB_W "RGB_Image_Viewer"
#define DEPTH_W "DEPTH_IMAGE_Viewer"
#define DEPTH_THRESHOLD "DEPTH_THRESHOLD"
#define RGB_THRESHOLD "RGB_THRESHOLD"

const double camera_factor = 1;

// #define SUBSCRIBE_TOPIC_RGB "/kinect2/rgb"
// #define SUBSCRIBE_TOPIC_DEPTH "/kinect2/depth"
// const double camera_cx = 2.5738909838479350e+02;
// const double camera_cy = 2.0617431757438302e+02;
// const double fx = 3.6095753862475351e+02;
// const double fy = 3.6068889959341760e+02;

const double ox = -310.373, oy = -303.158, oz = 1212.61;
double x_r, y_r, z_r;

/* simulate */
#define SUBSCRIBE_TOPIC_RGB "/kinect/rgb/image_raw"
#define SUBSCRIBE_TOPIC_DEPTH "/kinect/depth/image_raw"
image_transport::Publisher rviz_pub;
// const double camera_cx = 256.5;
// const double camera_cy = 212.5;
// const double fx = 443.4037529529496;
// const double fy = 443.4037529529496;

const double camera_cx = 320.5;
const double camera_cy = 240.5;
const double fx = 554.254691191187;
const double fy = 554.254691191187;

Rect rect(170, 10, 300, 300);
int blockSize = 5;
int constValue = 3;
int flag = 0;
/* target postion*/
int pixel_x = 0, pixel_y = 0;
float x, y, z;
bool _rgb_readed = false;

ros::Publisher target_pub;

cv::Mat lookupX, lookupY;

void createLookUp(int width, int height)
{
    lookupY = cv::Mat(1, height, CV_32FC1);
    float *it = lookupY.ptr<float>();
    for (int r = 0; r < height; r++, it++)
    {
        *it = float(r - camera_cy) / fy;
    }

    lookupX = cv::Mat(1, width, CV_32FC1);
    it = lookupX.ptr<float>();
    for (int c = 0; c < width; c++, it++)
    {
        *it = (c - camera_cx) / fx;
    }
}

bool compute_coordinate(int &m, int &n, float &depth_value)
{
    depth_value /= camera_factor;
    x = (m - camera_cx) * depth_value / fx;
    y = (n - camera_cy) * depth_value / fy;
    // x = lookupX.at<float>(0, m) * depth_value;
    // y = lookupX.at<float>(0, n) * depth_value;
    z = depth_value;

    x_r = (y - oy) + 484.675568;
    y_r = (x - ox) - 225.562668;
    z_r = (z - oz) + 294.332642;

    image_segment::target tg;
    // tg.x = 0.115121;
    // tg.y = -0.00998887;
    // tg.z = 1.13501;
    // tg.x = x_r;
    // tg.y = y_r;
    // tg.z = z_r;

    tg.x = x;
    tg.y = y;
    tg.z = z;

    ROS_INFO("x_r = %f, y_r = %f, z_r = %f \n", x_r, y_r, z_r);

    target_pub.publish(tg);
    return true;
}

void get_center_of_target(const kuka_image_processing::predictConstPtr &predict)
{
    //get the center of the segment
    pixel_x = int(predict->position) % 300 + 170;
    pixel_y = (int(predict->position) - pixel_x) / 300 + 10;
    // int cols = depth_threshold.cols;
    // int rows = depth_threshold.rows;
    // int count = 0, _cols_sum = 0, _rows_sum = 0, pixel = 0;
    // for (int i = 0; i < rows; ++i)
    //     for (int j = 0; j < cols; ++j)
    //     {
    //         pixel = depth_threshold.at<uchar>(i, j);
    //         if (pixel == 255)
    //         {
    //             count++;
    //             _cols_sum += j;
    //             _rows_sum += i;
    //         }
    //     }
    // pixel_y = _rows_sum / count + 135;
    // pixel_x = _cols_sum / count + 140;
}

int depth_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_INFO("in depth_callback");
    try
    {
        Mat depth = cv_bridge::toCvShare(msg, msg->encoding)->image;
        if (depth.empty())
            return 0;
        Mat depth_8UC1, depth_threshold;
        /* (depth-1000)*0.8 = depth*0.8-800*/
        // depth(rect).convertTo(depth_8UC1, CV_8UC1, 0.8, -800.0);

        /* simulate */
        // (depth-1.0)*255+100 = depth*255-155*/
        depth(rect).convertTo(depth_8UC1, CV_8UC1, 255.0, -155.0);
        // std::cout << depth(rect) << std::endl;

        // cv::adaptiveThreshold(depth_8UC1, depth_threshold, 2000.0, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);

        /* simulate */
        cv::adaptiveThreshold(depth_8UC1, depth_threshold, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);

        // medianBlur(depth_threshold, depth_threshold, 3);

        // get_center_of_target(depth_threshold);

        // cout << depth(rect) << endl;
        // flip(depth, depth, 1);

        if (!compute_coordinate(pixel_x, pixel_y, depth.at<float>(pixel_y, pixel_x)))
        {
            std::cerr << "compute_coordinate failed" << std::endl;
            return -1;
        }

        rectangle(depth, rect, Scalar(0, 0, 255), 2, 8);

        // imshow(DEPTH_W, depth / 3.0f);
        // imshow(DEPTH_THRESHOLD, depth_threshold);

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
        Mat rgb = cv_bridge::toCvShare(msg, msg->encoding)->image;

        /* simulate */
        // cvtColor(rgb, rgb, COLOR_RGB2BGR);

        // ROS_INFO("%s", msg->encoding.c_str());
        if (rgb.empty())
            return -1;
        // Mat roi = rgb(rect);
        rectangle(rgb, rect, Scalar(0, 0, 255), 2, 8);

        // Mat gray, _threshold;
        // cvtColor(roi, gray, COLOR_BGR2GRAY);
        // cv::adaptiveThreshold(gray, _threshold, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, blockSize, constValue);
        // medianBlur(_threshold, _threshold, 3);
        // imshow(RGB_THRESHOLD, _threshold);

        Rect rect_sign(pixel_x - 2, pixel_y - 2, 5, 5);
        rectangle(rgb, rect_sign, Scalar(0, 0, 255), 1, 8);
        string pos = "(" + to_string(x) + ", " + to_string(y) + ", " + to_string(z) + ")";
        putText(rgb, pos, Point2i(pixel_x - 200, pixel_y - 30), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 0), 2, 8);

        // imshow(RGB_W, rgb);
        sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb).toImageMsg();
        rviz_pub.publish(msg_rgb);

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
    ros::Subscriber predict_sub = nh.subscribe<kuka_image_processing::predict>("/predict", 1, get_center_of_target);
    createLookUp(512, 424);

    // namedWindow(RGB_W, WINDOW_AUTOSIZE);
    // namedWindow(DEPTH_W, WINDOW_AUTOSIZE);
    // namedWindow(DEPTH_THRESHOLD, WINDOW_NORMAL);
    // namedWindow(RGB_THRESHOLD, WINDOW_NORMAL);
    // namedWindow("threshold", WINDOW_AUTOSIZE);
    // namedWindow("grayImage", WINDOW_AUTOSIZE);
    // startWindowThread();
    rviz_pub = it.advertise("/target_pos",1);
    image_transport::Subscriber sub_depth = it.subscribe(SUBSCRIBE_TOPIC_DEPTH, 1, depth_callback);
    image_transport::Subscriber sub_rgb = it.subscribe(SUBSCRIBE_TOPIC_RGB, 1, rgb_callback);
    ros::spin();

    destroyAllWindows();
    return 0;
}
