#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>

#define pi 3.1415926
#define max_velocity 1
#define max_acceleration 1

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

void move(float x, float y, float z, float roll, float pitch, float yaw)
{
    moveit::planning_interface::MoveGroup group("manipulator"); //ur5对应moveit中选择的规划部分

    geometry_msgs::Pose target_pose; //设置发送的数据，对应于moveit中的拖拽
    tf::Quaternion Q;

    roll = roll / 180.0 * pi;
    pitch = pitch / 180.0 * pi;
    yaw = yaw / 180.0 * pi;
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

    moveit::planning_interface::MoveGroup::Plan my_plan; //进行运动规划，计算机器人移动到目标的运动轨迹，对应moveit中的plan按钮
    bool success = group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success)
        group.execute(my_plan); //让机械臂按照规划的轨迹开始运动，对应moveit中的execute。
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "movo_moveit");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    go_home();
    move(-0.143, 0.589, 1.75, 0, 0, 0);
    grasp(0.6);
    grasp(0.8);

    // ros::ServiceClient client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    // gazebo_msgs::SetModelState set_model_state_srv;
    // gazebo_msgs::ModelState des_model_state;
    // geometry_msgs::Twist twist;

    // twist.linear.x = 0.0;
    // twist.linear.y = 0.0;
    // twist.linear.z = 0.0;
    // twist.angular.x = 0.0;
    // twist.angular.y = 0.0;
    // twist.angular.z = 0.0;

    // geometry_msgs::Pose pose;
    // geometry_msgs::Quaternion quat;

    // tf::Quaternion Q;
    // Q.setRPY(0, 0, 0);

    // quat.x = Q.getX();
    // quat.y = Q.getY();
    // quat.z = Q.getZ();
    // quat.w = Q.getW();

    // pose.orientation = quat;
    // pose.position.x = -0.1;
    // pose.position.y = 0.7;
    // pose.position.z = 1.1;

    // des_model_state.model_name = "aruco";
    // des_model_state.pose = pose;
    // des_model_state.twist = twist;
    // des_model_state.reference_frame = "world";

    // set_model_state_srv.request.model_state = des_model_state;
    // client.call(set_model_state_srv);

    ros::shutdown();
    return 0;
}