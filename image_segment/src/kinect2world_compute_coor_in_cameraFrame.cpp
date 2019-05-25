#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DEPTH_WIN "DEPTH WIN"
#define RGB_WIN "RGB WIN"

std::vector<cv::Mat> depths(10);
std::vector<cv::Mat> rgbs(10);
int x = 0, y = 0, count = 1;
double wx = 0.0, wy = 0.0, wz = 0.0;

/* depth camera */
const double camera_factor = 1.0;
const double camera_cx = 2.5738909838479350e+02;
const double camera_cy = 2.0617431757438302e+02;
const double fx = 3.6095753862475351e+02;
const double fy = 3.6068889959341760e+02;



cv::Mat show;

bool compute_coordinate(int &m, int &n, float &depth_value)
{
    depth_value /= camera_factor;
    wx = (m - camera_cx) * depth_value / fx;
    wy = (n - camera_cy) * depth_value / fy;
    wz = depth_value;
    return true;
}

void on_track_x(int, void *)
{
    rgbs[count - 1].copyTo(show);
    cv::Rect rect(x - 2, y - 2, 5, 5);
    if (show.empty())
        return;
    cv::rectangle(show, rect, cv::Scalar(0, 0, 255), 1, 8);
    cv::imshow(RGB_WIN, show);
}

void on_track_y(int, void *)
{
    rgbs[count - 1].copyTo(show);
    cv::Rect rect(x - 2, y - 2, 5, 5);
    if (show.empty())
        return;
    cv::rectangle(show, rect, cv::Scalar(0, 0, 255), 1, 8);
    cv::imshow(RGB_WIN, show);
}

int main(int argc, char **argv)
{

    cv::namedWindow(RGB_WIN, cv::WINDOW_NORMAL);
    cv::namedWindow(DEPTH_WIN, cv::WINDOW_AUTOSIZE);

    cv::FileStorage fs("/home/nimpng/kinect2World.xml", cv::FileStorage::READ);

    int x_max, y_max;

    for (; count <= 10; count++)
    {
        fs[("depth" + std::to_string(count))] >> depths[count - 1];
        fs[("rgb" + std::to_string(count))] >> rgbs[count - 1];
        x_max = rgbs[count - 1].cols - 1;
        y_max = rgbs[count - 1].rows - 1;
        cv::createTrackbar("x", RGB_WIN, &x, x_max, on_track_x);
        cv::createTrackbar("y", RGB_WIN, &y, y_max, on_track_y);
        cv::imshow(RGB_WIN, rgbs[count - 1]);
        cv::imshow(DEPTH_WIN, depths[count - 1] / 4500.0f);
        int key = cv::waitKey(0);
        if (key == 's')
        {
            cv::FileStorage fs("target_pos.xml", cv::FileStorage::APPEND);
            ROS_INFO("%f\n", depths[count - 1].at<float>(y + 135, x + 140));
            if (!compute_coordinate(x, y, depths[count - 1].at<float>(y + 135, x + 140)))
                std::cerr << "error in computing coordinate" << std::endl;
            fs << ("target_position" + std::to_string(count)) << cv::Scalar(wx, wy, wz);
        }
    }
    fs.release();
    return 0;
}