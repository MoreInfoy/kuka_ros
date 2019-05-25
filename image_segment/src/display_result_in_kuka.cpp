#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <kuka_image_processing/predict.h>
#include <image_segment/target.h>

using namespace std;
using namespace cv;

#define SUBSCRIBE_TOPIC_RGB "/kinect2/rgb"
#define SUBSCRIBE_TOPIC_DEPTH "/kinect2/depth"
#define RGB_W "RGB_Image_Viewer"
#define DEPTH_W "DEPTH_Image_Viewer"
image_transport::Publisher rviz_pub_sd;
image_transport::Publisher rviz_pub_pr;
ros::Publisher target_pub;

float position = 0;
float angle = 0;
float width = 0;
float length = 10.0;
double angle_depth = 0.0;
Rect rect(135, 100, 100, 100);
Rect rect_d(135, 100, 100, 100);

void predict_callback(const kuka_image_processing::predictConstPtr &pred)
{
    image_segment::target tg;
    position = pred->position;
    angle = pred->angle + 3.1415926 / 2.0;
    // angle = angle_depth + 3.1415926 / 2.0;
    width = pred->width*10;

    tg.angle = angle;
    target_pub.publish(tg);
}

void check(Point2i &p)
{
    if (p.x < 0 || p.x >= 300)
        p.x = 0;
    if (p.y < 0 || p.y >= 300)
        p.y = 0;
}

void rgb_callback(const sensor_msgs::ImageConstPtr &image)
{
    try
    {
        Mat rgb = cv_bridge::toCvShare(image, image->encoding)->image(rect);
        if (rgb.empty())
            return;
        // FileStorage fs("/home/nimpng/kuka_ros/src/sendmsg.xml", FileStorage::READ);
        // Mat img;
        // fs["rgb_image"] >> img;
        // draw rectangle
        cvtColor(rgb, rgb, COLOR_BGRA2BGR);

        float diagonal = sqrt(pow(width / 2.0, 2) + pow(length / 2.0, 2));
        float par_angle = atan(length / width);
        float angle_u = angle + par_angle;
        float angle_d = angle - par_angle;
        int col = (int(position) % 300) / 3;
        int row = ((int(position) - col) / 300) / 3;
        Point2i p_u1(col, row), p_u2(col, row), p_d1(col, row), p_d2(col, row);
        int par_u1 = int(diagonal * cos(angle_u));
        int par_u2 = int(diagonal * sin(angle_u));
        Point2i par_u(par_u1, par_u2);

        int par_d1 = int(diagonal * cos(angle_d));
        int par_d2 = int(diagonal * sin(angle_d));
        Point2i par_d(par_d1, par_d2);

        p_u1 = p_u1 + par_u;
        p_u2 = p_u2 - par_u;
        p_d1 = p_d1 + par_d;
        p_d2 = p_d2 - par_d;

        cout << p_u1 << p_u2 << p_d1 << p_d2 << endl;

        check(p_u1);
        check(p_u2);
        check(p_d1);
        check(p_d2);

        line(rgb, p_u1, p_d1, Scalar(0, 0, 255), 1);
        line(rgb, p_u1, p_d2, Scalar(0, 0, 255), 1);
        line(rgb, p_u2, p_d1, Scalar(0, 0, 255), 1);
        line(rgb, p_u2, p_d2, Scalar(0, 0, 255), 1);

        // imshow(RGB_W, rgb);
        Mat rgb_resize;
        resize(rgb, rgb_resize, Size(300, 300), (0,0), (0,0), INTER_CUBIC);
        imwrite("/home/nimpng/Pictures/kuka_pre_cv.png", rgb_resize);
        sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_resize).toImageMsg();
        rviz_pub_pr.publish(msg_rgb);

        // int key = waitKey(30);
        // if (key == 's')
        // {
        //     imwrite("a_ggcnn_p.png", rgb);
        // }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("this %s", e.what());
    }
    return;
}

void depth_callback(const sensor_msgs::ImageConstPtr &image)
{
    Mat depth = cv_bridge::toCvShare(image, image->encoding)->image(rect_d);
    if (depth.empty())
        return;
    // cout << depth << endl;
    Mat roi_image, roi_threshold;
    Rect rect_f;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    convertScaleAbs(depth, roi_image, 0.8, -800);
    adaptiveThreshold(roi_image, roi_threshold, 1260.0, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 11, 7);
    medianBlur(roi_threshold, roi_threshold, 3);
    findContours(roi_threshold, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    cout << contours.size() << endl;
    Mat rgb_depth;
    cvtColor(roi_image, rgb_depth, COLOR_GRAY2BGR);
    Point2f vertex[4];
    float par = 10000000000;
    for (int i = 0; i < contours.size(); i++)
    {
        RotatedRect minRect = minAreaRect(contours[i]);
        minRect.points(vertex);
        int col = (int(position) % 300) / 3;
        int row = ((int(position) - col) / 300) / 3;
        Point2f dis(minRect.center.x - col, minRect.center.y - row);
        float dis_par = dis.dot(dis);
        if (dis_par < par)
        {
            par = dis_par;
            Point2f y_min, x_min, x_max;
            y_min = vertex[0];
            x_min = vertex[1];
            x_max = vertex[3];
            for (int j = 0; j < 4; j++)
            {
                if (vertex[j].x < x_min.x)
                    x_min = vertex[j];
                if (vertex[j].y < y_min.y)
                    y_min = vertex[j];
                if (vertex[j].x > x_max.x)
                    x_max = vertex[j];
                // line(rgb_depth, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);

                // rect_f = boundingRect(contours[i]);
                // rectangle(rgb_depth, rect_f, Scalar(0, 0, 255));
            }
            double par1 = pow(x_min.x - y_min.x, 2) + pow(x_min.y - y_min.y, 2);
            double par2 = pow(x_max.x - y_min.x, 2) + pow(x_max.y - y_min.y, 2);
            // double angle_d;
            if (par1 > par2)
            {
                angle_depth = atan((x_min.y - y_min.y) / (x_min.x - y_min.x));
            }
            else
                angle_depth = atan((x_max.y - y_min.y) / (x_max.x - y_min.x));
            cout << angle_depth << endl;
        }
        for (int j = 0; j < 4; j++)
        {
            line(rgb_depth, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);

            // rect_f = boundingRect(contours[i]);
            // rectangle(rgb_depth, rect_f, Scalar(0, 0, 255));
        }
    }
    // imshow(DEPTH_W, rgb_depth);
    sensor_msgs::ImagePtr msg_depth = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_depth).toImageMsg();
    rviz_pub_sd.publish(msg_depth);
    // waitKey(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "display_predict");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // namedWindow(RGB_W, WINDOW_NORMAL);
    // namedWindow(DEPTH_W, WINDOW_NORMAL);

    // startWindowThread();

    ros::Subscriber pre_sub = nh.subscribe<kuka_image_processing::predict>("/predict", 1, predict_callback);
    image_transport::Subscriber sub_rgb = it.subscribe(SUBSCRIBE_TOPIC_RGB, 1, rgb_callback);
    image_transport::Subscriber sub_depth = it.subscribe(SUBSCRIBE_TOPIC_DEPTH, 1, depth_callback);
    rviz_pub_sd = it.advertise("/segment_depth", 1);
    rviz_pub_pr = it.advertise("/predict_result", 1);
    target_pub = nh.advertise<image_segment::target>("/predict_angle", 1);
    ros::spin();

    destroyAllWindows();
    return 0;
}