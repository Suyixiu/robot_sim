#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/GraspConfigList.h>

#include <iostream>

using namespace std;

gpd_ros::GraspConfig best_grasp; //最好的抓取点的msg
// pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>); //点云格式指针 原始的全部点云
// pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);		//分割后的抓取物体的点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //点云格式指针 原始的全部点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);	  //分割后的抓取物体的点云
bool receive_gresp_flag = 1;															  //收到grasp消息

// char dir[] = "./src/robot_sim/experiment/grasp/GPD_method/pcd_save/test_gpd%d.pcd";
char dir[] = "./src/robot_sim/experiment/grasp/pcd_save/test_gpd%d.pcd";

gpd_ros::CloudSamples *pre_data()
{
	// 这里的CloudSamples这个类型包括
	// 1、一个自定义类型		gpd_ros/CloudSources cloud_sources
	// 2、一个点类型的数组		geometry_msgs/Point[] samples
	// 而这个cloud_sources的话则是又包括
	// 1、一个ROS下面的点云格式			    sensor_msgs/PointCloud2 cloud
	// 2、这个不知道是什么 直接填0就可以了	  std_msgs/Int64[] camera_source
	// 3、相机的位置 直接000就好了          geometry_msgs/Point[] view_points
	gpd_ros::CloudSamples *CloudSamples_msg = new gpd_ros::CloudSamples(); //要发送的sample

	sensor_msgs::PointCloud2 full_cloud_PointCloud2;
	pcl::toROSMsg(*full_cloud, full_cloud_PointCloud2);							   //转成ROS中的点云格式
	full_cloud_PointCloud2.header.frame_id = "camera_ir_optical_frame";			   //相对坐标
	CloudSamples_msg->cloud_sources.cloud = full_cloud_PointCloud2;				   //原始点云
	CloudSamples_msg->cloud_sources.view_points.push_back(geometry_msgs::Point()); //(0， 0， 0)
	std_msgs::Int64 Int64_zero;
	Int64_zero.data = 0;
	for (int i = 0; i < full_cloud->points.size(); i++)
	{
		CloudSamples_msg->cloud_sources.camera_source.push_back(Int64_zero); //camera_source都填为0
		/* 0.435 = 1.3998-0.9647 1.3998为ir相机的高度 0.9647为桌子的高度 */
		if (fabs(full_cloud->points[i].z - 0.723) > 0.0008)
		{
			// pcl::PointXYZ temp;
			pcl::PointXYZRGB temp;

			temp.x = full_cloud->points[i].x;
			temp.y = full_cloud->points[i].y;
			temp.z = full_cloud->points[i].z;
			temp.r = full_cloud->points[i].r;
			temp.g = full_cloud->points[i].g;
			temp.b = full_cloud->points[i].b;
			output->points.push_back(temp);
			geometry_msgs::Point sample_point;
			sample_point.x = temp.x;
			sample_point.y = temp.y;
			sample_point.z = temp.z;
			CloudSamples_msg->samples.push_back(sample_point);
		}
	}
	return CloudSamples_msg;
}

void graspCB(const gpd_ros::GraspConfigList &msg)
{
	if (!receive_gresp_flag)
	{
		best_grasp = msg.grasps[0];
		receive_gresp_flag = 1;
	}
}

int main(int argc, char **argv)
{
	setlocale(LC_ALL, "");
	ros::init(argc, argv, "open_a_file_and_pub_sample");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10); //设置发送数据的频率

	ros::Subscriber grasps_pub_ = nh.subscribe("/detect_grasps/clustered_grasps", 10, graspCB);
	ros::Publisher pc_sample_pub = nh.advertise<gpd_ros::CloudSamples>("cloud_sample", 1); //用于发送抓取物体的点云的index
	ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_output", 1);		   //用于发送分割后的点云

	while (pc_sample_pub.getNumSubscribers() < 1)
		;
	while (pc_pub.getNumSubscribers() < 1)
		;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	static int pcl_file_index = 1;

	while (ros::ok())
	{
		if (receive_gresp_flag)
		{
			ros::Duration(1.0).sleep(); //停一秒用于观察
			string pack_path = ros::package::getPath("robot_sim");
			string pcd_path = pack_path + "/grasp/pcd_save/test_gpd%d.pcd";
			char save_dir[100];
			sprintf(save_dir, pcd_path.c_str(), pcl_file_index); //装填路径
			ROS_INFO("\033[1;32m 载入%s \033[0m", save_dir);
			pcl::io::loadPCDFile(save_dir, *full_cloud);		  //载入点云文件
			gpd_ros::CloudSamples *CloudSamples_msg = pre_data(); //装填CloudSamples用于发布的消息 而且做个简单的分割
			sensor_msgs::PointCloud2 seg_reslut;				  //ROS格式的点云分割结果
			pcl::toROSMsg(*output, seg_reslut);					  //分割结果转成ROS中的点云格式
			output->clear();
			seg_reslut.header.frame_id = "camera_ir_optical_frame"; //点云的frame
			pc_pub.publish(seg_reslut);								//发布分割结果到 pc_output 这个话题
			pc_sample_pub.publish(*CloudSamples_msg);				//发布CloudSamples的消息给gpd_ros
			delete CloudSamples_msg;
			ROS_INFO("\033[1;32m CloudSamples_msg发布完毕 \033[0m");
			receive_gresp_flag = 0;
			pcl_file_index++;
			if (pcl_file_index == 5)
				pcl_file_index = 1;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}