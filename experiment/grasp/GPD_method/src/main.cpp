#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <gpd_ros/CloudIndexed.h>
#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/GraspConfigList.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/server.h>
#include <robot_sim/gpdConfig.h>

#include <tf/transform_listener.h>

#include <iostream>

#define math_pi 3.14159265

bool tf_flag = 0;
bool at_home = 0;

using namespace std;

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

// obj_pos_t box = {"box", -0.13, 0.75, 1.2, 0, 0, 100};
// obj_pos_t coke = {"coke", -0.25, 0.88, 1, 90, 30, 20};
// obj_pos_t banana = {"banana", -0.1, 0.65, 1.2, 90, 0, 100};
// obj_pos_t dropbox = {"dropbox", -0.5, 0.25, 1.2, 0, 0, 90};
// obj_pos_t wooden_peg = {"wooden_peg", -0.25, 0.73, 1.2, 0, 0, -20};

// obj_pos_t box = {"box", -0.18, 0.75, 1.1, 0.0, 0, 75.6};
// obj_pos_t coke = {"coke", -0.3, 0.78, 1.1, 90, 30, 140.4};
// obj_pos_t banana = {"banana", -0.1, 0.87, 1.1, 90, 0, 21.6};
// obj_pos_t dropbox = {"dropbox", -0.5, 0.25, 1.2, 0, 0, 90};
// obj_pos_t wooden_peg = {"wooden_peg", -0.31, 0.71, 1.2, 0, 0, -20};

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

bool move(float x, float y, float z, float roll, float pitch, float yaw)
{
	moveit::planning_interface::MoveGroupInterface group("manipulator"); //ur5对应moveit中选择的规划部分

	geometry_msgs::Pose target_pose; //设置发送的数据，对应于moveit中的拖拽
	tf::Quaternion Q;

	roll = roll / 180.0 * math_pi;
	pitch = pitch / 180.0 * math_pi;
	yaw = yaw / 180.0 * math_pi;
	Q.setRPY(roll, pitch, yaw);
	target_pose.orientation.x = Q.getX();
	target_pose.orientation.y = Q.getY();
	target_pose.orientation.z = Q.getZ();
	target_pose.orientation.w = Q.getW();

	target_pose.position.x = x;
	target_pose.position.y = y;
	target_pose.position.z = z;

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
	gripper_group.setMaxVelocityScalingFactor(0.3);
	gripper_group.setMaxAccelerationScalingFactor(1);
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

// string fcloud_rame_id = "camera_rgb_optical_frame";
string fcloud_rame_id = "camera_ir_optical_frame";
sensor_msgs::PointCloud2 seg_reslut; //分割后的点云转成ROS格式
// pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>); //点云格式指针 原始的全部点云
// pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);		//分割后的抓取物体的点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //点云格式指针 原始的全部点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);	  //分割后的抓取物体的点云
sensor_msgs::PointCloud2 full_cloud_PointCloud2;										  //ROS中点云消息格式的变量
gpd_ros::CloudSamples *CloudSamples_msg;												  //要发送的sample
vector<int> cloud_index;																  //要抓的东西的点云的index
bool update_flag = 0;																	  //收到点云后的标志位

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

void cloudCB(const sensor_msgs::PointCloud2 &pcl_msg)
{
	/* 只有在原点的地方才处理点云 */
	if (at_home)
	{
		// 这里的CloudSamples这个类型包括
		// 1、一个自定义类型		gpd_ros/CloudSources cloud_sources
		// 2、一个点类型的数组		geometry_msgs/Point[] samples
		// 而这个cloud_sources的话则是又包括
		// 1、一个ROS下面的点云格式			    sensor_msgs/PointCloud2 cloud
		// 2、这个不知道是什么 直接填0就可以了	  std_msgs/Int64[] camera_source
		// 3、相机的位置 直接000就好了          geometry_msgs/Point[] view_points
		gpd_ros::CloudSamples *gpd_msg = new gpd_ros::CloudSamples(); //要发送的sample

		sensor_msgs::PointCloud2 full_cloud_PointCloud2 = pcl_msg;			  //ros类型的消息直接就进CloudSamples的cloud_sources.cloud
		full_cloud_PointCloud2.header.frame_id = fcloud_rame_id;			  //相对坐标
		gpd_msg->cloud_sources.cloud = full_cloud_PointCloud2;				  //原始点云
		gpd_msg->cloud_sources.view_points.push_back(geometry_msgs::Point()); //(0， 0， 0)

		pcl::fromROSMsg(pcl_msg, *full_cloud); //从ROS类型消息转为PCL类型消息
		std_msgs::Int64 Int64_zero;
		Int64_zero.data = 0; //去nan
		output->clear();
		for (size_t i = 0; i < full_cloud->points.size(); i++)
		{
			gpd_msg->cloud_sources.camera_source.push_back(Int64_zero); //camera_source都填为0
			/* 0.435 = 1.3998-0.9647 1.3998为ir相机的高度 0.9647为桌子的高度 */
			if (fabs(full_cloud->points[i].z - 0.7233) > 0.005)
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
				gpd_msg->samples.push_back(sample_point);
				// cloud_index.push_back(i);
			}
		}
		CloudSamples_msg = gpd_msg;
		printf("segmentation done!\n");
		at_home = 0;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_gpd");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10); //设置发送数据的频率

	ros::Subscriber pc_input_sub = nh.subscribe("/camera/depth/points", 10, cloudCB);
	// ros::Subscriber pc_input_sub = nh.subscribe("/camera/rgb/points", 10, cloudCB);
	ros::Subscriber grasps_sub_ = nh.subscribe("/detect_grasps/clustered_grasps", 10, graspCB); //GPD发布的grasp
	ros::Publisher pc_sample_pub = nh.advertise<gpd_ros::CloudSamples>("cloud_sample", 1);		//用于发送抓取物体的点云的index
	ros::Publisher seg_result_pub = nh.advertise<sensor_msgs::PointCloud2>("pc_output", 1);

	ROS_INFO("let's rock!\n");

	// while (pc_sample_pub.getNumSubscribers() < 1)
	// 	;
	// while (seg_result_pub.getNumSubscribers() < 1)
	// 	;

	dynamic_reconfigure::Server<robot_sim::gpdConfig> server;
	dynamic_reconfigure::Server<robot_sim::gpdConfig>::CallbackType callback;

	callback = boost::bind(&gpd_dynamic_callback, _1);
	server.setCallback(callback);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	grasp(0.6);
	grasp(0.8);
	go_home();
	reset_obj_pos();
	ros::Duration(1).sleep();

	tf::TransformListener listener;
	/* 首先获取机械臂初始位置时world到tool的TF关系 */
	tf::StampedTransform world2tool;
	while (1)
	{
		try
		{
			listener.lookupTransform("/world", "/yixiuge_ee_link", ros::Time(0), world2tool);
			printf("world2tool %f  %f  %f  %f  %f  %f %f\n", world2tool.getOrigin().x(), world2tool.getOrigin().y(), world2tool.getOrigin().z(),
				   world2tool.getRotation().getX(), world2tool.getRotation().getY(), world2tool.getRotation().getZ(), world2tool.getRotation().getW());
			break; //如果成功了就跳出去咯
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s", ex.what());
			ROS_ERROR("lookupTransform world to yixiuge_ee_link faild", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
	}

	/* 最好的那个grasp的pose变成ur爪子的坐标系有两个旋转 */
	tf::Transform grasp2urPose;
	tf::Quaternion grasp2urPose_Q;
	grasp2urPose_Q.setRPY(math_pi / 2, 0, -math_pi / 2);
	grasp2urPose.setRotation(grasp2urPose_Q);
	grasp2urPose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

	/* 手到眼的TF 由手眼标定而来 */
	// tf::Transform tool2cam;
	// tf::Quaternion tool2cam_Q(0.999992, -0.002807, -0.002813, 0.000013);
	// tool2cam.setRotation(tool2cam_Q);
	// tool2cam.setOrigin(tf::Vector3(-0.037471, 0.127866, 0.136702));
	/* 手到深度相机的的TF */
	tf::Transform tool2cam;
	tf::Quaternion tool2cam_Q(1, -0.0, -0.0, 0.0);
	tool2cam.setRotation(tool2cam_Q);
	tool2cam.setOrigin(tf::Vector3(-0.0325, 0.128, 0.1425));

	/* world到cam的关系 只要你起始位置也就是world2tool是固定的那world2cam就一定是固定的 */
	tf::Transform world2cam;
	world2cam.mult(world2tool, tool2cam); //world2tool X tool2cam = world2cam;

	/* 开始第一次直接就是发点云咯 */
	at_home = 1;
	while (at_home) //等待处理完点云分割出物体
		;
	pcl::toROSMsg(*output, seg_reslut);
	cout << "output points size\t" << output->points.size() << endl;
	seg_reslut.header.frame_id = fcloud_rame_id;
	seg_result_pub.publish(seg_reslut);
	pc_sample_pub.publish(*CloudSamples_msg);
	delete CloudSamples_msg;
	cout << "publish finish!\n";

	while (ros::ok())
	{
		static int count = 0;
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

			tf::Transform cam2urPose, world2urPose;
			cam2urPose.mult(cam2grasp, grasp2urPose); //cam2grasp X grasp2urPose = cam2urPose;
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

			world2urPose.getOrigin().setZ(world2urPose.getOrigin().z() + 0.01);

			tf::Transform above_10cm_of_best, add_10cm_to_Z;
			tf::Quaternion add_10cm_to_Z_Q;
			add_10cm_to_Z_Q.setRPY(0, 0, 0);
			add_10cm_to_Z.setRotation(add_10cm_to_Z_Q);
			add_10cm_to_Z.setOrigin(tf::Vector3(0.0, 0.0, 0.10));
			above_10cm_of_best.mult(world2urPose, add_10cm_to_Z);

			move(above_10cm_of_best);
			move(world2urPose);
			/* 抓取并扔到盒子里 */
			grasp(0.33);
			go_home();
			move(-0.465, 0.24, 1.4, 0.0, 0.0, 0.0); //箱子上边
			grasp(0.8);
			count++;
			if (count == 3)
			{
				count = 0;
				reset_obj_pos();
			}
			// ros::Duration(20).sleep();//暂停10秒用于截图
			go_home();

			ros::Duration(0.5).sleep();
			/* 回到home了那就发点云咯 */
			at_home = 1;
			while (at_home) //等待处理完点云
				;
			pcl::toROSMsg(*output, seg_reslut);
			seg_reslut.header.frame_id = fcloud_rame_id;
			seg_result_pub.publish(seg_reslut);
			pc_sample_pub.publish(*CloudSamples_msg);
			delete CloudSamples_msg;
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
			seg_reslut.header.frame_id = fcloud_rame_id;
			seg_result_pub.publish(seg_reslut);
			pc_sample_pub.publish(*CloudSamples_msg);
			delete CloudSamples_msg;
			cout << "publish finish!\n";
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}