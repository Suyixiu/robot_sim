#include "control.h"

#define math_pi 3.1415926

#define max_velocity 1
#define max_acceleration 1

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
