#include "control.h"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define math_pi 3.14159265
#define max_velocity 2.0
#define max_acceleration 2.0

cv::Point3f home(-0.2209, 0.7, 0.2 + 1.2);

double pose_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
	double distance = sqrt(pow(pose1.position.x - pose2.position.x, 2) +
						   pow(pose1.position.y - pose2.position.y, 2) +
						   pow(pose1.position.z - pose2.position.z, 2));
	return distance;
}

/**
* @brief	控制机械臂的move函数
* @param	x：从上往下看右边是x的正方向        ^ Y
* @param	y：从上往下看前边是y的正方向        |
* @param	z：从上往下看上边是z的正方向        + —— > X
* @param	roll：正中是0 具体需要角度自己看坐标轴旋转一下就知道了
* @param	pitch：正中是0 具体需要角度自己看坐标轴旋转一下就知道了
* @param	yaw：从上往下看 顺时针从左到右分别是90 0 -90
*/
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

/**
* @brief	控制机械臂的move函数
* @param	target_point:目标点
* @param	yaw：从上往下看 顺时针从左到右分别是90 0 -90
*/
bool move(cv::Point3f target_point, float yaw)
{
	moveit::planning_interface::MoveGroupInterface group("manipulator"); //ur5对应moveit中选择的规划部分

	geometry_msgs::Pose target_pose; //设置发送的数据，对应于moveit中的拖拽
	tf::Quaternion Q;

	yaw = yaw / 180.0 * math_pi;
	Q.setRPY(0.0, 0.0, yaw);
	target_pose.orientation.x = Q.getX();
	target_pose.orientation.y = Q.getY();
	target_pose.orientation.z = Q.getZ();
	target_pose.orientation.w = Q.getW();

	target_pose.position.x = target_point.x;
	target_pose.position.y = target_point.y;
	target_pose.position.z = target_point.z;

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

bool move(float x, float y, float z, float qx, float qy, float qz, float qw)
{
	moveit::planning_interface::MoveGroupInterface group("manipulator"); //ur5对应moveit中选择的规划部分

	geometry_msgs::Pose target_pose; //设置发送的数据，对应于moveit中的拖拽

	target_pose.orientation.x = qx;
	target_pose.orientation.y = qy;
	target_pose.orientation.z = qz;
	target_pose.orientation.w = qw;

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

bool line_move(tf::Transform target_tf, double velScale, double accScale)
{
	moveit::planning_interface::MoveGroupInterface group("manipulator");
	geometry_msgs::Pose final_pose; //最后一个点

	group.setGoalJointTolerance(0.001);
	group.setMaxAccelerationScalingFactor(accScale);
	group.setMaxVelocityScalingFactor(velScale);

	moveit::core::RobotStatePtr start_state(group.getCurrentState()); // 获取机器人的起始位置
	const robot_state::JointModelGroup *joint_model_group = start_state->getJointModelGroup(group.getName());

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose target_pose_temp;
	target_pose_temp.position.x = target_tf.getOrigin().x();
	target_pose_temp.position.y = target_tf.getOrigin().y();
	target_pose_temp.position.z = target_tf.getOrigin().z();
	target_pose_temp.orientation.x = target_tf.getRotation().getX();
	target_pose_temp.orientation.y = target_tf.getRotation().getY();
	target_pose_temp.orientation.z = target_tf.getRotation().getZ();
	target_pose_temp.orientation.w = target_tf.getRotation().getW();
	waypoints.push_back(target_pose_temp);

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
	int maxtries = 3; //最大尝试规划次数
	int attempts = 0; //已经尝试规划次数

	while (fraction < 1.0 && attempts < maxtries)
	{
		fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");

		// 生成机械臂的运动规划数据
		moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
		robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "manipulator");
		rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		iptp.computeTimeStamps(rt, velScale, accScale);

		rt.getRobotTrajectoryMsg(trajectory);
		joinedPlan.trajectory_ = trajectory;
		// 执行运动
		group.execute(joinedPlan);
		return 1;
		// sleep(1);
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return 0;
	}
}


bool continuous_move(std::vector<tf::Transform> tf_sets)
{
	moveit::planning_interface::MoveGroupInterface group("manipulator"); //ur5对应moveit中选择的规划部分

	std::vector<geometry_msgs::Pose> waypoints;
	// geometry_msgs::Pose currentPose = group.getCurrentPose().pose;
	// waypoints.push_back(currentPose);

	for (int i = 0; i < tf_sets.size(); i++)
	{
		geometry_msgs::Pose target_pose_temp;
		target_pose_temp.position.x = tf_sets[i].getOrigin().x();
		target_pose_temp.position.y = tf_sets[i].getOrigin().y();
		target_pose_temp.position.z = tf_sets[i].getOrigin().z();
		target_pose_temp.orientation.x = tf_sets[i].getRotation().getX();
		target_pose_temp.orientation.y = tf_sets[i].getRotation().getY();
		target_pose_temp.orientation.z = tf_sets[i].getRotation().getZ();
		target_pose_temp.orientation.w = tf_sets[i].getRotation().getW();
		waypoints.push_back(target_pose_temp);
	}

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
	ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
	int maxtries = 3; //最大尝试规划次数
	int attempts = 0; //已经尝试规划次数

	while (fraction < 1.0 && attempts < maxtries)
	{
		fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");

		// 生成机械臂的运动规划数据
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;

		// 执行运动
		group.execute(plan);
		return 1;
		// sleep(1);
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return 0;
	}
}

void go_home(void)
{
	moveit::planning_interface::MoveGroupInterface group("manipulator");
	robot_state::RobotState current_state(*group.getCurrentState());
	current_state.setToDefaultValues(current_state.getJointModelGroup("manipulator"), "yixiuge_home");
	// current_state.setToDefaultValues(current_state.getJointModelGroup("manipulator"), "test");
	group.setJointValueTarget(current_state);
	group.setMaxVelocityScalingFactor(max_velocity);
	group.setMaxAccelerationScalingFactor(max_acceleration);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	if (success)
		group.execute(my_plan);
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
	gripper_group.setMaxVelocityScalingFactor(max_velocity);
	gripper_group.setMaxAccelerationScalingFactor(max_acceleration);

	moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
	bool grasp_flag = gripper_group.plan(gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	if (grasp_flag)
		gripper_group.execute(gripper_plan);
}

void move_obj(string obj_name, float x, float y, float z, float roll, float pitch, float yaw)
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
	roll = roll / 180.0 * math_pi;
	pitch = pitch / 180.0 * math_pi;
	yaw = yaw / 180.0 * math_pi;
	Q.setRPY(roll, pitch, yaw);

	quat.x = Q.getX();
	quat.y = Q.getY();
	quat.z = Q.getZ();
	quat.w = Q.getW();

	pose.orientation = quat;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;

	des_model_state.model_name = obj_name;
	des_model_state.pose = pose;
	des_model_state.twist = twist;
	des_model_state.reference_frame = "world";

	set_model_state_srv.request.model_state = des_model_state;
	client.call(set_model_state_srv);
	std_msgs::Bool flag;
	flag.data = true;
	pub_update_flag.publish(flag);
}

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
