#ifndef _gpd_interface_H_
#define _gpd_interface_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>  //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/GraspConfigList.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include "control.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <chrono>
#include <iostream>

using namespace Eigen;
using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class TransformSender
{
public:
    ros::NodeHandle node_;
    //constructor
    TransformSender(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string &frame_id, const std::string &child_frame_id)
    {
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x, y, z)), time, frame_id, child_frame_id);
    };
    TransformSender(double x, double y, double z, double qx, double qy, double qz, double qw, ros::Time time, const std::string &frame_id, const std::string &child_frame_id) : transform_(tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z)), time, frame_id, child_frame_id){};
    //Clean up ros connections
    ~TransformSender() {}

    //A pointer to the rosTFServer class
    tf::TransformBroadcaster broadcaster;

    // A function to call to send data periodically
    void send(ros::Time time)
    {
        transform_.stamp_ = time;
        broadcaster.sendTransform(transform_);
    };

private:
    tf::StampedTransform transform_;
};

extern PointCloudT::Ptr full_cloud;
extern PointCloudT::Ptr output;
extern PointCloudT::Ptr cloud_hull;

extern string pc_output_topic;
extern string pc_input_topic; //点云所在的话题
extern string fram_id;	 //相机坐标系

extern gpd_ros::GraspConfig best_grasp;
extern sensor_msgs::PointCloud2 seg_reslut;
extern gpd_ros::CloudSamples CloudSamples_msg;
extern bool receive_grasp_flag;
extern bool at_home;

void cloudCB(const sensor_msgs::PointCloud2 &msg);
void graspCB(const gpd_ros::GraspConfigList &msg);

#endif