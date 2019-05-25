#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <kuka_coordinate_transform/toolpose.h>

#define DEPTH_WIN "DEPTH WIN"
#define RGB_WIN "RGB WIN"
#define DEPTH_TOPIC "/kinect/depth/image_raw"
#define RGB_TOPIC "/kinect/rgb/image_raw"
#define WIDTH int(512)
#define HEIGHT int(424)

int x = 0, y = 0;
double wx = 0.0, wy = 0.0, wz = 0.0;

cv::Mat lookupX, lookupY;
cv::Mat show, rgb, depth;

cv::FileStorage fs("/home/nimpng/coor_data.xml", cv::FileStorage::APPEND);
int count = 42;
bool saved_pt_in_camera = false;

/* depth camera */
const double camera_factor = 1.0;
// const double camera_cx = 2.5738909838479350e+02;
// const double camera_cy = 2.0617431757438302e+02;
// const double fx = 3.6095753862475351e+02;
// const double fy = 3.6068889959341760e+02;

const double camera_cx = 256.5;
const double camera_cy = 212.5;
const double fx = 443.4037529529496;
const double fy = 443.4037529529496;

/* color camera */
// const double camera_factor = 1.0;
// const double camera_cx = 9.7068961309905160e+02;
// const double camera_cy = 5.7063611432534515e+02;
// const double fx = 1.1038810564083312e+03;
// const double fy = 1.1025482697948601e+03;

bool compute_coordinate(int &m, int &n, float &depth_value)
{
    // depth_value /= camera_factor;
    // std::cout << lookupX.at<float>(0, m) << ", " << lookupY.at<float>(0, m) << std::endl;
    wx = lookupX.at<double>(0, m) * depth_value;
    wy = lookupY.at<double>(0, n) * depth_value;

    // wx = (m - camera_cx) * depth_value / fx;
    // wy = (n - camera_cy) * depth_value / fy;
    wz = depth_value;
    return true;
}
void depth_callback(const sensor_msgs::ImageConstPtr &image)
{
    depth = cv_bridge::toCvShare(image, image->encoding)->image;
    // std::cout << depth << std::endl;
    cv::Rect rect(x - 2, y - 2, 5, 5);
    cv::rectangle(depth, rect, cv::Scalar(0, 0, 255), 1, 8);
    cv::imshow(DEPTH_WIN, depth / 3.0f);
    int key = cv::waitKey(300);
    if (key == 'p')
    {
        compute_coordinate(x, y, depth.at<float>(y, x));
        std::cout << "x = " << wx
                  << ", y = " << wy
                  << ", z = " << wz
                  << ", count = " << count
                  << std::endl;
        fs << ("pt_in_camera_" + std::to_string(count)) << (cv::Mat_<double>(1, 3) << wx, wy, wz);
        saved_pt_in_camera = true;
    }
}

void rgb_callback(const sensor_msgs::ImageConstPtr &image)
{
    rgb = cv_bridge::toCvShare(image, image->encoding)->image;
    cv::Rect rect(x - 2, y - 2, 5, 5);
    cv::rectangle(rgb, rect, cv::Scalar(0, 0, 255), 1, 8);
    cv::imshow(RGB_WIN, rgb);
}

void callback(const kuka_coordinate_transform::toolposeConstPtr &pt)
{
    if (saved_pt_in_camera)
    {
        saved_pt_in_camera = false;
        fs << ("pt_in_camera_" + std::to_string(count))
           << (cv::Mat_<double>(1, 6) << pt->X, pt->Y, pt->Z, pt->A, pt->B, pt->C);
        count ++;
    }
}

void on_track_x(int, void *)
{
    // rgb.copyTo(show);
    // cv::Rect rect(x - 2, y - 2, 5, 5);
    // if (show.empty())
    //     return;
    // cv::rectangle(rgb, rect, cv::Scalar(0, 0, 255), 1, 8);
    // cv::imshow(RGB_WIN, rgb);
}

void on_track_y(int, void *)
{
    // rgb.copyTo(show);
    // cv::Rect rect(x - 2, y - 2, 5, 5);
    // if (show.empty())
    //     return;
    // cv::rectangle(rgb, rect, cv::Scalar(0, 0, 255), 1, 8);
    // cv::imshow(RGB_WIN, show);
}

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect2robot");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    cv::namedWindow(RGB_WIN, cv::WINDOW_NORMAL);
    cv::namedWindow(DEPTH_WIN, cv::WINDOW_NORMAL);

    cv::startWindowThread();

    cv::createTrackbar("x", RGB_WIN, &x, WIDTH, on_track_x);
    cv::createTrackbar("y", RGB_WIN, &y, HEIGHT, on_track_y);

    createLookUp(WIDTH, HEIGHT);

    image_transport::Subscriber sub_depth = it.subscribe(DEPTH_TOPIC, 1, depth_callback);
    image_transport::Subscriber sub_rgb = it.subscribe(RGB_TOPIC, 1, rgb_callback);
    ros::Subscriber sub = nh.subscribe<kuka_coordinate_transform::toolpose>("kuka_tool_coordinate", 1, callback);

    ros::spin();

    fs.release();
    return 0;
}