#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#define DEPTH_WIN "DEPTH WIN"
#define RGB_WIN "RGB WIN"

#define POINT_NUM int(50)

/* compute transform matrix from kinect to robot */

double camera_factor = 1000.0;
const double camera_cx = 2.5738909838479350e+02;
const double camera_cy = 2.0617431757438302e+02;
const double fx = 3.6095753862475351e+02;
const double fy = 3.6068889959341760e+02;

// cv::Mat lookupX, lookupY;

// void createLookUp(int width, int height)
// {
//     lookupY = cv::Mat(1, height, CV_32FC1);
//     float *it = lookupY.ptr<float>();
//     for (int r = 0; r < height; r++, it++)
//     {
//         *it = float(r - camera_cy) / fy;
//     }

//     lookupX = cv::Mat(1, width, CV_32FC1);
//     it = lookupX.ptr<float>();
//     for (int c = 0; c < width; c++, it++)
//     {
//         *it = (c - camera_cx) / fx;
//     }
// }

bool getmat(const cv::Mat &mat_in, cv::Mat &X, const int &index)
{
    X.at<double>(0, index) = mat_in.at<double>(0, 0);
    X.at<double>(1, index) = mat_in.at<double>(0, 1);
    X.at<double>(2, index) = mat_in.at<double>(0, 2);
    // std::cout << X.at<double>(0, index) << ", " << X.at<double>(1, index) << ", " << X.at<double>(2, index) << std::endl;
    return true;
}

bool mat2pt(const cv::Mat &mat_in, std::vector<cv::Point3d> &pt, const int &index)
{
    pt[index].x = mat_in.at<double>(0, 0);
    pt[index].y = mat_in.at<double>(0, 1);
    pt[index].z = mat_in.at<double>(0, 2);
    return true;
}

void pose_estimation_3d3d(
    const std::vector<cv::Point3d> &pts1,
    const std::vector<cv::Point3d> &pts2,
    cv::Mat &R, cv::Mat &t)
{
    cv::Point3d p1, p2; // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++)
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = cv::Point3d(cv::Vec3d(p1) / N);
    p2 = cv::Point3d(cv::Vec3d(p2) / N);
    std::vector<cv::Point3d> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++)
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++)
    {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    std::cout << "W=" << W << std::endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix<double, 3, 2> U_ = U.block(0, 0, 3, 2);
    Eigen::Matrix<double, 3, 2> V_ = V.block(0, 0, 3, 2);

    std::cout << "singularVal=" << svd.singularValues() << std::endl;

    std::cout << "U=" << U_ << std::endl;
    std::cout << "V=" << V_ << std::endl;

    Eigen::Matrix3d R_ = (U * (V.transpose())).transpose();
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    R = (cv::Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2),
         R_(1, 0), R_(1, 1), R_(1, 2),
         R_(2, 0), R_(2, 1), R_(2, 2));
    t = (cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

int main(int argc, char **argv)
{

    cv::FileStorage fs("/home/nimpng/coor_data_new.xml", cv::FileStorage::READ);
    cv::FileStorage fs_result("/home/nimpng/pos_transform_matrix_new.xml", cv::FileStorage::WRITE);

    std::vector<cv::Mat> cr_in_camera(POINT_NUM), cr_in_robot(POINT_NUM);
    std::vector<cv::Point3d> pts1(POINT_NUM), pts2(POINT_NUM);

    /* X*B = Y */
    cv::Mat X(cv::Size(POINT_NUM, 4), CV_64FC1);
    cv::Mat Y(cv::Size(POINT_NUM, 3), CV_64FC1);

    // createLookUp(512, 424);

    cv::Mat x;
    double par;
    for (int count = 0; count < POINT_NUM; count++)
    {

        fs[("pt_in_camera_" + std::to_string(count))] >> cr_in_camera[count];
        // std::cout << cr_in_camera[count] << std::endl;
        getmat(cr_in_camera[count], X, count);
        // par = cr_in_camera[count].at<double>(0, 0) / cr_in_camera[count].at<double>(0, 2);
        // for (int i = 0; i < 512; i++)
        // {
        //     if(par - lookupX.at<float>(0, i) < 0.0001 && par - lookupX.at<float>(0, i) > -0.0001) std::cout << "i=" << i << std::endl;
        // }
        // for (int  j= 0; j < 424; j++)
        // {
        //     if(par - lookupY.at<float>(0, j) < 0.0001 && par - lookupY.at<float>(0, j) > -0.0001) std::cout << "j=" << j << std::endl;
        // }
        // std::cout << "________________" << std::endl;
        mat2pt(cr_in_camera[count], pts1, count);
        X.at<double>(3, count) = 1.0;
        fs[("pt_in_kuka_" + std::to_string(count))] >> cr_in_robot[count];
        getmat(cr_in_robot[count], Y, count);
        mat2pt(cr_in_robot[count], pts2, count);
    }
    // std::cout << X << std::endl;
    // std::cout << Y << std::endl;
    // cv::Mat CR_M;
    // cv::merge(cr_in_camera, CR_M);
    fs.release();
    fs_result << "X" << X;
    fs_result << "Y" << Y;
    // cv::imwrite("/home/nimpng/cv_m.tiff", X);

    // cv::Mat X_trans;
    // cv::transpose(X, X_trans);

    // cv::Mat par = X_trans * X;
    // cv::Mat RT = par.inv() * X_trans * Y;

    cv::Mat R, t;
    pose_estimation_3d3d(pts2, pts1, R, t);

    std::cout << "R = " << R << std::endl;
    std::cout << "T = " << t << std::endl;

    fs_result << "R" << R;
    fs_result << "T" << t;

    // // std::cout << "matRT = " << RT << std::endl;

    // // fs_result << "matRT" << RT;
    cv::Mat RT = (cv::Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                  R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                  R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    // fs_result << "RT" << RT;

    cv::Mat error = RT * X - Y;

    std::cout << "error mat = " << error << std::endl;
    std::cout << "error = " << error.dot(error / 1000.0) / 1000.0 << std::endl;
    fs_result.release();

    return 0;
}