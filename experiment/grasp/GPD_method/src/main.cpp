#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <gpd_ros/CloudIndexed.h>
#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/GraspConfigList.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/server.h>
#include <robot_sim/gpdConfig.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>

using namespace Eigen;

#define math_pi 3.14159265

bool tf_flag = 0;
bool at_home = 1;

using namespace std;

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

#define max_velocity 2.0
#define max_acceleration 2.0

bool add_pi = false;
bool good_grasp = false;
bool bad_grasp = false;

void gpd_dynamic_callback(robot_sim::gpdConfig &config)
{
	add_pi = config.add_pi;
	good_grasp = config.good_grasp;
	bad_grasp = config.bad_grasp;
}

typedef struct
{
	const char *obj_name;
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
} obj_pos_t;

void move_obj(obj_pos_t obj_pos)
{
	ros::NodeHandle node_handle;
	ros::Publisher pub_update_flag = node_handle.advertise<std_msgs::Bool>("pos_update_flag", 1);
	ros::ServiceClient client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	gazebo_msgs::SetModelState set_model_state_srv;
	gazebo_msgs::ModelState des_model_state;
	geometry_msgs::Twist twist;

	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = 0.0;

	geometry_msgs::Pose pose;
	geometry_msgs::Quaternion quat;

	tf::Quaternion Q;
	obj_pos.roll = obj_pos.roll / 180.0 * math_pi;
	obj_pos.pitch = obj_pos.pitch / 180.0 * math_pi;
	obj_pos.yaw = obj_pos.yaw / 180.0 * math_pi;
	Q.setRPY(obj_pos.roll, obj_pos.pitch, obj_pos.yaw);

	quat.x = Q.getX();
	quat.y = Q.getY();
	quat.z = Q.getZ();
	quat.w = Q.getW();

	pose.orientation = quat;
	pose.position.x = obj_pos.x;
	pose.position.y = obj_pos.y;
	pose.position.z = obj_pos.z;

	des_model_state.model_name = obj_pos.obj_name;
	des_model_state.pose = pose;
	des_model_state.twist = twist;
	des_model_state.reference_frame = "world";

	set_model_state_srv.request.model_state = des_model_state;
	client.call(set_model_state_srv);
	std_msgs::Bool flag;
	flag.data = true;
	pub_update_flag.publish(flag);
}

obj_pos_t box = {"box", -0.13, 0.75, 1.2, 0, 0, 126};
obj_pos_t coke = {"coke", -0.25, 0.88, 1, 90, 30, 10};
obj_pos_t banana = {"banana", -0.1, 0.65, 1.2, 90, 0, 126};
obj_pos_t dropbox = {"dropbox", -0.5, 0.25, 1.2, 0, 0, 90};
obj_pos_t wooden_peg = {"wooden_peg", -0.25, 0.73, 1.2, 90, 0, -20};
obj_pos_t obj_pos[] = {box, coke, banana, dropbox, wooden_peg};
void reset_obj_pos()
{
	for (int i = 0; i < 5; i++)
	{
		move_obj(obj_pos[i]);
	}
}

bool move(tf::Transform target_tf)
{
	moveit::planning_interface::MoveGroupInterface group("manipulator"); //ur5对应moveit中选择的规划部分

	geometry_msgs::Pose target_pose; //设置发送的数据，对应于moveit中的拖拽

	target_pose.orientation.x = target_tf.getRotation().getX();
	target_pose.orientation.y = target_tf.getRotation().getY();
	target_pose.orientation.z = target_tf.getRotation().getZ();
	target_pose.orientation.w = target_tf.getRotation().getW();

	target_pose.position.x = target_tf.getOrigin().x();
	target_pose.position.y = target_tf.getOrigin().y();
	target_pose.position.z = target_tf.getOrigin().z();

	group.setPoseTarget(target_pose);
	group.setMaxVelocityScalingFactor(max_velocity);
	group.setMaxAccelerationScalingFactor(max_acceleration);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan; //进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
	bool success = group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

	ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	if (success)
	{
		group.execute(my_plan); //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
		return 1;
	}
	else
	{
		return 0;
	}
}

void grasp(float angle)
{
	/* 抓取 */
	moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

	std::vector<double> rbq_joint_values;
	gripper_group.getCurrentState()->copyJointGroupPositions(gripper_group.getCurrentState()->getRobotModel()->getJointModelGroup(gripper_group.getName()), rbq_joint_values); // 通过设置关节值的方式。

	/* robotiq共有六个joint，只有设置gripper_finger1_joint才能生效，对应下面的rbq_joint_values[2] */
	rbq_joint_values[0] = 0;
	rbq_joint_values[1] = 0;
	rbq_joint_values[2] = 0.8f - angle; // your joint value
	rbq_joint_values[3] = 0;
	rbq_joint_values[4] = 0;
	rbq_joint_values[5] = 0;

	gripper_group.setJointValueTarget(rbq_joint_values);
	moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
	bool grasp_flag = gripper_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	if (grasp_flag)
		gripper_group.execute(gripper_plan);
}

void go_home(void)
{
	moveit::planning_interface::MoveGroupInterface group("manipulator");
	robot_state::RobotState current_state(*group.getCurrentState());
	current_state.setToDefaultValues(current_state.getJointModelGroup("manipulator"), "yixiuge_home");
	group.setJointValueTarget(current_state);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	if (success)
		group.execute(my_plan);
}

sensor_msgs::PointCloud2 seg_reslut;												//分割后的点云转成ROS格式
pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>); //点云格式指针 原始的全部点云
pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);		//分割后的抓取物体的点云
sensor_msgs::PointCloud2 full_cloud_PointCloud2;									//ROS中点云消息格式的变量
gpd_ros::CloudIndexed CloudIndexed_msg;												//要发送的index
gpd_ros::CloudSamples CloudSamples_msg;												//要发送的sample
vector<int> cloud_index;															//要抓的东西的点云的index
bool update_flag = 0;																//收到点云后的标志位

gpd_ros::GraspConfig best_grasp; //最好的抓取点的msg
bool receive_grasp_flag = 0;	 //收到grasp消息

void graspCB(const gpd_ros::GraspConfigList &msg)
{
	cout << "receive grasp\n";
	if (!receive_grasp_flag)
	{
		best_grasp = msg.grasps[0];
		receive_grasp_flag = 1;
	}
}

void cloudCB(const sensor_msgs::PointCloud2 &msg)
{
	/* 只有在原点的地方才处理点云 */
	if (at_home)
	{
		/* clear一下 */
		CloudSamples_msg.cloud_sources.view_points.clear();

		output->points.clear();
		CloudSamples_msg.samples.clear();

		// PointCloudT::Ptr temp(new PointCloudT);
		pcl::fromROSMsg(msg, *full_cloud); //从ROS类型消息转为PCL类型消息

		// pcl::PointCloud<pcl::PointXYZRGBA> cloud;
		// pcl::fromROSMsg(msg, cloud); //从ROS类型消息转为PCL类型消息
		// pcl::io::savePCDFileASCII("./1.pcd", cloud); //保存原始的pcd

		// removeNan(temp, full_cloud);																				//去nan
		for (size_t i = 0; i < full_cloud->points.size(); i++)
		{
			/* 0.7233 = 1.688-0.9647 1.688为ir相机的高度 0.9647为桌子的高度 */
			if (fabs(full_cloud->points[i].z - 0.7233) > 0.005)
			{
				pcl::PointXYZ temp;

				temp.x = full_cloud->points[i].x;
				temp.y = full_cloud->points[i].y;
				temp.z = full_cloud->points[i].z;
				output->points.push_back(temp);
				geometry_msgs::Point sample_point;
				sample_point.x = output->points[i].x;
				sample_point.y = output->points[i].y;
				sample_point.z = output->points[i].z;
				CloudSamples_msg.samples.push_back(sample_point);
				// cloud_index.push_back(i);
			}
		}
		sensor_msgs::PointCloud2 full_cloud_PointCloud2 = msg;						  //ros类型的消息直接就进CloudSamples_msg.cloud_sources.cloud
		full_cloud_PointCloud2.header.frame_id = "camera_ir_optical_frame";			  //相对坐标
		CloudSamples_msg.cloud_sources.cloud = full_cloud_PointCloud2;				  //原始点云
		CloudSamples_msg.cloud_sources.view_points.push_back(geometry_msgs::Point()); //(0， 0， 0)

		CloudSamples_msg.cloud_sources.camera_source.clear();
		std_msgs::Int64 Int64_zero;
		Int64_zero.data = 0;
		for (int i = 0; i < full_cloud->points.size(); i++)
		{
			CloudSamples_msg.cloud_sources.camera_source.push_back(Int64_zero);
		}
		printf("segmentation done!\n");
		at_home = 0;
	}
}

void pre_data()
{
	/* 因为是文件打开所以点云数量等东西是直接固定的 */
	sensor_msgs::PointCloud2 full_cloud_PointCloud2;
	pcl::toROSMsg(*full_cloud, full_cloud_PointCloud2); //转成ROS中的点云格式
	// full_cloud_PointCloud2.header.frame_id = "base_link";						  //相对坐标
	full_cloud_PointCloud2.header.frame_id = "camera_ir_optical_frame";			  //相对坐标
	CloudSamples_msg.cloud_sources.cloud = full_cloud_PointCloud2;				  //原始点云
	CloudSamples_msg.cloud_sources.view_points.push_back(geometry_msgs::Point()); //(0， 0， 0)
	std_msgs::Int64 Int64_zero;
	Int64_zero.data = 0;
	/* 填一个全部点云大小的零 */
	for (int i = 0; i < full_cloud->points.size(); i++)
	{
		CloudSamples_msg.cloud_sources.camera_source.push_back(Int64_zero);
		if (fabs(full_cloud->points[i].z - 0.7233) > 0.005)
		{
			pcl::PointXYZ temp;

			temp.x = full_cloud->points[i].x;
			temp.y = full_cloud->points[i].y;
			temp.z = full_cloud->points[i].z;
			output->points.push_back(temp);
			geometry_msgs::Point sample_point;
			sample_point.x = temp.x;
			sample_point.y = temp.y;
			sample_point.z = temp.z;
			CloudSamples_msg.samples.push_back(sample_point);
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_gpd");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10); //设置发送数据的频率

	// ros::Subscriber pcl_sub = nh.subscribe("/camera/depth/points", 10, cloudCB);			//将/kinect2/depth/points这个topic的点云保存下来
	ros::Publisher pcl_sample_pub = nh.advertise<gpd_ros::CloudSamples>("cloud_sample", 1); //用于发送抓取物体的点云的index
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);		//用于发送分割后的点云

	// while (pcl_sample_pub.getNumSubscribers() < 1)
	// 	;
	// while (pcl_pub.getNumSubscribers() < 1)
	// 	;

	dynamic_reconfigure::Server<robot_sim::gpdConfig> server;
	dynamic_reconfigure::Server<robot_sim::gpdConfig>::CallbackType callback;

	callback = boost::bind(&gpd_dynamic_callback, _1);
	server.setCallback(callback);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ROS_INFO("let's rock!\n");

	pcl::io::loadPCDFile("./1.pcd", *full_cloud);
	/* 因为是打开文件 所以直接把里面的cloud跟index都确定下来 */
	pre_data();
	/* 开始第一次直接就是发点云咯 */
	sensor_msgs::PointCloud2 seg_reslut;
	pcl::toROSMsg(*output, seg_reslut);
	seg_reslut.header.frame_id = "camera_ir_optical_frame";
	pcl_pub.publish(seg_reslut);
	pcl_sample_pub.publish(CloudSamples_msg);
	cout << "publish finish!\n";
	grasp(0.6);
	grasp(0.8);
	go_home();
	reset_obj_pos();
	ros::Duration(1).sleep();

	tf::Transform world2cam;
	tf::Quaternion world2cam_Q(1.0, 0.0, 0.0, 0.0);
	world2cam_Q.setRPY(0, 0, math_pi);
	world2cam.setRotation(world2cam_Q);
	world2cam.setOrigin(tf::Vector3(-0.196355, 0.801140, 1.687952));

	/* 最好的那个grasp的pose变成ur爪子的坐标系 */
	tf::Transform grasp2urPose;
	tf::Quaternion grasp2urPose_Q;
	grasp2urPose_Q.setRPY(math_pi / 2, 0, -math_pi / 2);
	grasp2urPose.setRotation(grasp2urPose_Q);
	grasp2urPose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

	/* 开始第一次直接就是发点云咯 */
	at_home = 1;
	while (at_home) //等待处理完点云分割出物体
		;
	pcl::toROSMsg(*output, seg_reslut);
	cout << "output points size\t" << output->points.size() << endl;
	seg_reslut.header.frame_id = "camera_ir_optical_frame";
	pcl_pub.publish(seg_reslut);
	pcl_sample_pub.publish(CloudSamples_msg);
	cout << "publish finish!\n";

	while (ros::ok())
	{
		/* 如果收到爪子且是好的则让机械臂过去 */
		if (receive_grasp_flag && good_grasp)
		{
			receive_grasp_flag = 0;
			good_grasp = 0;
			tf::Matrix3x3 orientation(
				best_grasp.approach.x, best_grasp.binormal.x, best_grasp.axis.x,
				best_grasp.approach.y, best_grasp.binormal.y, best_grasp.axis.y,
				best_grasp.approach.z, best_grasp.binormal.z, best_grasp.axis.z);
			double roll, pitch, yaw;
			tf::Vector3 Bottom(best_grasp.position.x, best_grasp.position.y, best_grasp.position.z);
			tf::Vector3 final_point = Bottom + 0.025 * orientation.getColumn(0); //Approach

			tf::Transform cam2grasp;
			cam2grasp.setOrigin(final_point);
			tf::Quaternion cam2grasp_pose_Q;
			orientation.getRotation(cam2grasp_pose_Q); //将旋转矩阵转成四元数
			cam2grasp.setRotation(cam2grasp_pose_Q);   //设置cam2grasp的旋转

			tf::Transform cam2urPose, cam2urPose_top_10cm, world2urPose;
			cam2urPose.mult(cam2grasp, grasp2urPose); //cam2grasp X grasp2urPose = cam2urPose;
			/* 妈的如果要反转180度 */
			if (add_pi)
			{
				add_pi = 0;
				tf::Transform yaw_pi_rotation;
				tf::Quaternion yaw_pi_rotation_Q;
				yaw_pi_rotation_Q.setRPY(0, 0, math_pi);
				yaw_pi_rotation.setRotation(yaw_pi_rotation_Q);
				yaw_pi_rotation.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
				cam2urPose.mult(cam2urPose, yaw_pi_rotation);
			}
			world2urPose.mult(world2cam, cam2urPose); //world2cam X cam2urPose = world2urPose;
			tf::Matrix3x3(world2urPose.getRotation()).getRPY(roll, pitch, yaw);
			cout << roll * 180 / math_pi << "\t" << pitch * 180 / math_pi << "\t" << yaw * 180 / math_pi << "\n";
			if (yaw * 180 / math_pi > -90 && yaw * 180 / math_pi < 90)
			{
				std::cout << "dont need to add pi\n";
			}
			else
			{
				tf::Transform yaw_pi_rotation;
				tf::Quaternion yaw_pi_rotation_Q;
				yaw_pi_rotation_Q.setRPY(0, 0, math_pi);
				yaw_pi_rotation.setRotation(yaw_pi_rotation_Q);
				yaw_pi_rotation.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
				world2urPose.mult(world2urPose, yaw_pi_rotation);
			}

			tf::Transform above_10cm_of_best, add_10cm_to_Z;
			tf::Quaternion add_10cm_to_Z_Q;
			add_10cm_to_Z_Q.setRPY(0, 0, 0);
			add_10cm_to_Z.setRotation(add_10cm_to_Z_Q);
			add_10cm_to_Z.setOrigin(tf::Vector3(0.0, 0.0, 0.10));
			above_10cm_of_best.mult(world2urPose, add_10cm_to_Z);

			printf("rosrun yixiuge_ur move_test %f  %f  %f  %f  %f  %f %f\n", world2urPose.getOrigin().x(), world2urPose.getOrigin().y(), world2urPose.getOrigin().z(),
				   world2urPose.getRotation().getX(), world2urPose.getRotation().getY(), world2urPose.getRotation().getZ(), world2urPose.getRotation().getW());
			printf("rosrun tf static_transform_publisher %f  %f  %f  %f  %f  %f %f world wtf 1000\n", world2urPose.getOrigin().x(), world2urPose.getOrigin().y(), world2urPose.getOrigin().z(),
				   world2urPose.getRotation().getX(), world2urPose.getRotation().getY(), world2urPose.getRotation().getZ(), world2urPose.getRotation().getW());

			/* 去抓 */
			move(world2urPose);
			go_home();

			ros::Duration(0.5).sleep();
			/* 回到home了那就发点云咯 */
			at_home = 1;
			while (at_home) //等待处理完点云
				;
			pcl::toROSMsg(*output, seg_reslut);
			seg_reslut.header.frame_id = "camera_ir_optical_frame";
			pcl_pub.publish(seg_reslut);
			pcl_sample_pub.publish(CloudSamples_msg);
			cout << "publish finish!\n";
		}
		/* 收到爪子但不能去则待在原地重新发分割的点云 */
		else if (bad_grasp)
		{
			bad_grasp = 0;
			ros::Duration(1).sleep();
			at_home = 1;
			while (at_home) //等待处理完点云
				;
			pcl::toROSMsg(*output, seg_reslut);
			seg_reslut.header.frame_id = "camera_ir_optical_frame";
			pcl_pub.publish(seg_reslut);
			pcl_sample_pub.publish(CloudSamples_msg);
			cout << "publish finish!\n";
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}