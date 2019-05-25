#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
// #include <log4cxx/helpers/tchar.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;
using namespace cv;

/*-----camera parameters-----*/
const double fx = 3.6095753862475351e+02;
const double fy = 3.6068889959341760e+02;
const double cx = 2.5738909838479350e+02;
const double cy = 2.0617431757438302e+02;
const double camera_factor = 1000;

const double depthShift = -2.2796754425186673e+01;
const double k1k2Distortion[2] = {7.4228875873069922e-02, -2.3955825991485458e-01};
const double p1p2p3Distortion[3] = {-3.0993070617678850e-03, 8.7541958873480321e-04,
                              9.1495561545413773e-02};

/*----processor support------*/
enum Processor
{
    cl,
    gl,
    cpu
};

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
    protonect_shutdown = true;
}

void parametersInit(Mat &matrix, Mat &coeff)
{
    matrix.at<double>(0, 0) = fx;
    matrix.at<double>(0, 2) = cx;
    matrix.at<double>(1, 1) = fy;
    matrix.at<double>(1, 2) = cy;
    matrix.at<double>(2, 2) = 1.0;
    coeff.at<double>(0, 0) = k1k2Distortion[0];
    coeff.at<double>(0, 1) = k1k2Distortion[1];
    coeff.at<double>(0, 2) = p1p2p3Distortion[0];
    coeff.at<double>(0, 3) = p1p2p3Distortion[1];
    coeff.at<double>(0, 4) = p1p2p3Distortion[2];
}

bool createCloud(Mat &depth, Mat &rgb, PointCloud::Ptr &cloud)
{
#pragma omp paraller for
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            float d = depth.ptr<float>(m)[n];
            if (0 == d)
                continue;
            PointT p;
            p.z = double(d) / camera_factor;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;

            p.r = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.b = rgb.ptr<uchar>(m)[n * 3 + 2];
            p.a = 255;

            cloud->points.push_back(p);
            // std::cout << cloud->points.size() << std::endl;
        }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    // pcl::io::savePCDFileASCII("/home/nimpng/tets_pcd.txt", *cloud);
    return true;
}

bool deleteNan(Mat &src)
{
    int rows = src.rows;
    int cols = src.cols;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if (src.at<float>(i, j) == NAN || src.at<float>(i, j) < 0)
            {
                src.at<float>(i, j) = 0.0f;
            }
        }
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DepthImage_Publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_depth = it.advertise("kinect2/depth", 1);
    image_transport::Publisher pub_rgb = it.advertise("kinect2/rgb", 1);

    // camera intristic matrix
    Mat matrix = Mat::zeros(Size(3, 3), CV_64F);
    Mat coeff(1, 5, CV_64F);

    parametersInit(matrix, coeff);


    //! [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    //! [context]

    //! [discovery]
    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    if (serial == "")
        return -1;
    std::cout << "The serial number is: " << serial << std::endl;

    int depthProcessor = Processor::cl;

    if (depthProcessor == Processor::cpu)
    {
        if (!pipeline)
            //! [pipeline]
            pipeline = new libfreenect2::CpuPacketPipeline();
        //! [pipeline]
    }
    else if (depthProcessor == Processor::gl)
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if (!pipeline)
            pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
        std::cout << "OpenGL pipeline is not supported!" << endl;
#endif
    }
    else if (depthProcessor == Processor::cl)
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if (!pipeline)
            pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
        cout << "OpenCL pipeline is not supported!" << endl;
#endif
    }

    if (pipeline)
    {
        //! [open]
        dev = freenect2.openDevice(serial, pipeline);
        //! [open]
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if (dev == 0)
    {
        cout << "failure opening device!" << endl;
        return -1;
    }

    signal(SIGINT, sigint_handler);
    protonect_shutdown = false;

    //! [listeners]
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    //! [listeners]

    //! [start]
    dev->start();

    cout << "device serial: " << dev->getSerialNumber() << endl;
    cout << "device firmware: " << dev->getFirmwareVersion() << endl;
    //! [start]

    //! [registration setup]
    libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), bigdepth(1920, 1080 + 2, 4);
    //! [registration setup]

    Mat _rgb_mat, _depth_mat, _bigdepth_mat, irmat, _rgb_of_depth;
    Mat _depth_mat_undistorted, _rgb_of_depth_undistorted, undistortDepth, _new_matrix, _shift_depth;
    PointCloud::Ptr cloud(new PointCloud);

    // /*----create cloud viewer------*/
    // const std::string cloudName = "cloud viewer";
    // pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));

    // visualizer->addPointCloud(cloud, cloudName);
    // visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 1, cloudName);
    // visualizer->initCameraParameters();
    // visualizer->setBackgroundColor(0, 0, 0);
    // visualizer->setPosition(600, 0);
    // visualizer->setSize(1080, 800);
    // visualizer->setShowFPS(true);
    // visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);

    //! [loop start]
    while (!protonect_shutdown && ros::ok())
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        // std::cout << depth->height << ", " << depth->width << std::endl;
        //! [loop start]

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(_rgb_mat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(_depth_mat);

        // cv::imshow("rgb", _rgb_mat);
        // cv::imshow("depth", _depth_mat / 4500.0f);

        //! [registration]

        registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth);
        //! [registration]

        cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(_depth_mat_undistorted);
        cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(_rgb_of_depth);
        cv::Mat(bigdepth.height, bigdepth.width, CV_32FC1, bigdepth.data).copyTo(_bigdepth_mat);
        // cv::imshow("distorted", _depth_mat_undistorted / 4500.0f);
        // cv::imshow("registered", _rgb_of_depth);
        cv::imwrite("rgb_of_depth_dis.png", _rgb_of_depth);
        cv::imwrite("depth_dis.png", _depth_mat_undistorted / 10.0f);


        _depth_mat_undistorted.convertTo(_shift_depth, CV_32FC1, 1.0, depthShift);
        // cv::imshow("depth_shift", _shift_depth / 4500.0f);
        // cv::imwrite("depth_shift.tiff", _shift_depth / 4500.0f);
        // std::cout << _shift_depth << std::endl;
        // std::cout << matrix << "|| " << coeff << std::endl;
        undistort(_rgb_of_depth, _rgb_of_depth_undistorted, matrix, coeff, _new_matrix);
        undistort(_shift_depth, undistortDepth, matrix, coeff, _new_matrix);
        cv::imwrite("rgb_of_depth_undis.png", _rgb_of_depth_undistorted);
        cv::imwrite("depth_undis.png", undistortDepth / 10.0f);



        flip(_rgb_of_depth_undistorted, _rgb_of_depth_undistorted, 1);
        flip(undistortDepth, undistortDepth, 1);

        // cv::imwrite("kinect_bigdepth.tiff", _bigdepth_mat/4500.0f);
        // cv::imwrite("kinect_rgb.png", _rgb_mat);
        // cv::imwrite("kinect_rgb_depth.tiff", _rgb_of_depth);

        // cv::imwrite("kinect_depth_dis.tiff", _depth_mat_undistorted/100.0f);
        // cv::imwrite("kinect_depth.tiff", undistortDepth/100.0f);
        // cv::imwrite("kinect_registered.png", _rgb_of_depth_undistorted);
        // createCloud(undistortDepth, _rgb_of_depth_undistorted, cloud);
        // visualizer->updatePointCloud(cloud, cloudName);
        // bilateralFilter(_rgb_of_depth_undistorted, _rgb_of_depth_undistorted_filter, 5, 2.0, 2.0);
        // medianBlur(_rgb_of_depth_undistorted, _rgb_of_depth_undistorted, 3);
        // cv::imshow("depth_rect", undistortDepth / 4500.0f);
        // cv::imshow("depth_undistort", undistortDepth / 4500.0f);
        // cv::imshow("registered_undistorted", _rgb_of_depth_undistorted);

        // std::cout << undistortDepth << std::endl;
        sensor_msgs::ImagePtr msg_depth = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, undistortDepth).toImageMsg();
        sensor_msgs::ImagePtr msg_rgb = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC4, _rgb_of_depth_undistorted).toImageMsg();
        pub_depth.publish(msg_depth);
        pub_rgb.publish(msg_rgb);
        // cv::Mat image = cv_bridge::toCvShare(msg_depth, msg_depth->encoding)->image;
        // imwrite("kinectDepth.tiff", image);
        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        //! [loop end]
        listener.release(frames);
        // visualizer->spinOnce(10);
    }
    //! [loop end]

    // visualizer->close();

    //! [stop]
    std::cout << "stop: " << dev->stop() << std::endl;
    std::cout << "close: " << dev->close() << std::endl;

    //! [stop]

    delete registration;

    return 0;
}

