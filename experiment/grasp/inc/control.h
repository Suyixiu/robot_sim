#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Bool.h>

using namespace std;

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

bool move(cv::Point3f target_point, float yaw);
bool move(float x, float y, float z, float roll, float pitch, float yaw);
bool move(float x, float y, float z, float qx, float qy, float qz, float qw);
bool line_move(tf::Transform target_tf, double velScale, double accScale);
void grasp(float angle);
void move_obj(string obj_name, float x, float y, float z, float roll, float pitch, float yaw);
void reset_obj_pos();
void go_home(void);

#endif