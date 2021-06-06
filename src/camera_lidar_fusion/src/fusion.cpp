#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>
#include <boost/bind.hpp>
#include <opencv2/core/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <cmath>
#include <hash_map>
#include <ctime>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include "common.h"
#include <chrono>


using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

class CameraLidarFusion{
    typedef pcl::PointXYZRGB PointT;
private:
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_;
    string camera_topic_, lidar_topic_, intrinsic_path_, extrinsic_path_;
    message_filters::Subscriber<sensor_msgs::Image> camera_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    ros::Publisher colored_cloud_pub_;

public:
    CameraLidarFusion();
    ~CameraLidarFusion();
    void callback(const sensor_msgs::ImageConstPtr& input_image_msg, const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg);
    void getUV(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, float* UV);
    void getColor(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int* RGB);
};

CameraLidarFusion::CameraLidarFusion() : private_node_("~") {
    cout << "\033[1;32m Get the parameters from the launch file \033[0m" << endl;
    private_node_.getParam("camera_topic", camera_topic_);
    private_node_.getParam("lidar_topic", lidar_topic_);
    private_node_.getParam("intrinsic_path", intrinsic_path_);
    private_node_.getParam("extrinsic_path", extrinsic_path_);

    camera_sub_.subscribe(node_handle_, camera_topic_, 15);
    lidar_sub_.subscribe(node_handle_, lidar_topic_, 10);
    colored_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("colored_cloud_toshow", 10);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera_sub_, lidar_sub_);

    sync.registerCallback(boost::bind(&CameraLidarFusion::callback, this, _1, _2));
    ros::spin();

}

CameraLidarFusion::~CameraLidarFusion() {

}
void CameraLidarFusion::callback(const sensor_msgs::ImageConstPtr& input_image_msg, const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) {
    cout << "\033[1;32m Welcome to callback !!!!!!! \033[0m" << endl;
    vector<float> intrinsic;
    getIntrinsic(intrinsic_path_, intrinsic);
    vector<float> distortion;
    getDistortion(intrinsic_path_, distortion);
    vector<float> extrinsic;
    getExtrinsic(extrinsic_path_, extrinsic);

    double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]}, {intrinsic[3], intrinsic[4], intrinsic[5]}, {intrinsic[6], intrinsic[7], intrinsic[8]}};
    double matrix2[3][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]}, {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]}, {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}};

    cv::Mat matrix_in(3, 3, CV_64F, matrix1);
    cv::Mat matrix_out(3, 4, CV_64F, matrix2);

    cv::Mat camera_matrix = cv::Mat::eye(3, 3 ,CV_64F);
    camera_matrix.at<double>(0, 0) = intrinsic[0];
    camera_matrix.at<double>(0, 2) = intrinsic[2];
    camera_matrix.at<double>(1, 1) = intrinsic[4];
    camera_matrix.at<double>(1, 2) = intrinsic[5];

    cv::Mat distortion_coef = cv::Mat::zeros(5, 1, CV_64F);
    distortion_coef.at<double>(0, 0) = distortion[0];
    distortion_coef.at<double>(1, 0) = distortion[1];
    distortion_coef.at<double>(2, 0) = distortion[2];
    distortion_coef.at<double>(3, 0) = distortion[3];
    distortion_coef.at<double>(4, 0) = distortion[4];

    cv::Mat input_image;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
        return;
    }
    input_image = cv_ptr->image;

    cv::Mat view, rview, map1, map2;
    cv::Size imageSize = input_image.size();
    cv::initUndistortRectifyMap(camera_matrix, distortion_coef, cv::Mat(),cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coef, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    cv::remap(input_image, input_image, map1, map2, cv::INTER_LINEAR);  // correct the distortion

    int row = input_image.rows;
    int col = input_image.cols;
    //cout << row << endl;
    //cout << col << endl << endl;
    vector<vector<int>> color_vector;
    color_vector.resize(row*col);
    for (unsigned int i = 0; i < color_vector.size(); ++i) {
        color_vector[i].resize(3);
    }
    for (int v = 0; v < row; ++v) {
        for (int u = 0; u < col; ++u) {
            color_vector[v*col + u][0] = input_image.at<cv::Vec3b>(v, u)[2];
            color_vector[v*col + u][1] = input_image.at<cv::Vec3b>(v, u)[1];
            color_vector[v*col + u][2] = input_image.at<cv::Vec3b>(v, u)[0];
        }
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*input_cloud_msg, *output_cloud_msg);//very important
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = input_cloud_msg->width ;
    cloud->points.resize(cloud->width);
    double w_sigma = 0.07;
    cv::RNG rng;
    for(uint64_t i = 0; i < cloud->points.size(); ++i){
        float x = output_cloud_msg->points[i].x;
        float y = output_cloud_msg->points[i].y;
        float z = output_cloud_msg->points[i].z;
        if(x == 0 && y == 0 && z == 0) {
            continue;
        }
        cloud->points[i].x = x;
        cloud->points[i].y = y;
        cloud->points[i].z = z;
        int RGB[3] = {0, 0, 0};
        getColor(matrix_in, matrix_out, x, y, z, row, col, color_vector, RGB);
        if (RGB[0] == 0 && RGB[1] == 0 && RGB[2] == 0) {
            continue;
        }
        cloud->points[i].r = RGB[0];
        cloud->points[i].g = RGB[1];
        cloud->points[i].b = RGB[2];

        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*cloud, output_cloud);
        output_cloud.header = input_cloud_msg->header;
        output_cloud.header.frame_id = input_cloud_msg->header.frame_id;

        colored_cloud_pub_.publish(output_cloud);

    }

}
void CameraLidarFusion::getUV(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, float* UV){
    double matrix3[4][1] = {x, y, z, 1};
    cv::Mat coordinate(4, 1, CV_64F, matrix3);
    cv::Mat result = matrix_in*matrix_out*coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);
    UV[0] = u / depth;
    UV[1] = v / depth;

}
void CameraLidarFusion::getColor(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int* RGB) {
    float UV[2] = {0, 0};
    getUV(matrix_in, matrix_out, x, y, z, UV);
    int u = int(UV[0]);
    int v = int(UV[1]);
    int32_t index = v*col + u;
    if (index < row*col && index >= 0) {
        RGB[0] = color_vector[index][0];
        RGB[1] = color_vector[index][1];
        RGB[2] = color_vector[index][2];
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "fusion");

    CameraLidarFusion cameraLidarFusion;

    return 0;
}
