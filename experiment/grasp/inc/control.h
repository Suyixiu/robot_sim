#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Bool.h>

#include "recognize.h"

#include <math.h>
#include <iostream>
#include <string>

#include <stdio.h>

using namespace std;

#define math_pi 3.1415926

#define table_hight 0.996
#define max_velocity 1
#define max_acceleration 1

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

#define home_ID 0
#define pos1_ID 1
#define pos2_ID 2
#define pos3_ID 3
#define pos4_ID 4
#define test_ID 5

extern Point3f home;

bool move(Point3f target_point, float yaw);
bool move(float x, float y, float z, float roll, float pitch, float yaw);
bool move(float x, float y, float z, float qx, float qy, float qz, float qw);
bool move(tf::Transform target_tf);
bool line_move(tf::Transform target_tf, double velScale, double accScale);
void continuous_move(std::vector<Point3f> point_sets, std::vector<float> yaw_sets);
bool continuous_move(std::vector<tf::Transform> tf_sets);
bool continuous_move(std::vector<tf::Transform> tf_sets, double velScale, double accScale);
bool continuous_move_to_drop(tf::Transform above_10cm_of_best, double velScale, double accScale);
bool continuous_move_to_grasp(tf::Transform above_10cm_of_best, tf::Transform world2urPose, double velScale, double accScale);
void grasp(float angle);
void dh_grasp(float angle, float force);
void move_obj(string obj_name, float x, float y, float z, float roll, float pitch, float yaw);
void reset_obj_pos();
void go_home(void);
void go_fix_pose(uint point_ID);

#endif