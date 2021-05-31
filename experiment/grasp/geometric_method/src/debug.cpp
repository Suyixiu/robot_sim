#include "debug.h"

/* 参数全局变量 */
string obj_name = "banana";
double obj_x = 0.75;
double obj_y = 0.25;
double obj_z = 0.98;
double obj_roll = 0.0;
double obj_pitch = 0.0;
double obj_yaw = 0.0;
double eelink_x = 0.67;
double eelink_y = 0.11;
double eelink_z = -0.1;
double eelink_roll = 0.0;
double eelink_pitch = 0.0;
double eelink_yaw = -90.0;
double grasp_angle = 0.8;
bool enable_move_and_grasp_flag = false;
bool enable_move_obj_flag = false;
bool reset_obj_pos_flag = false;
bool enable_moveit_flag = false;
bool enable_grasp_flag = false;
bool save_image_flag = false;
bool save_mask_flag = false;

void grasp_dynamic_callback(robot_sim::graspConfig &config)
{
    // ROS_INFO("Reconfigure Request: %s %d %f %f %s %d",
    //          config.cmd_topic.c_str(),
    //          config.cmd_pub_rate,
    //          config.linear_x,
    //          config.angular_z,
    //          config.move_flag ? "True" : "False",
    //          config.log_level);

    obj_name = config.obj_name;
    obj_x = config.obj_x;
    obj_y = config.obj_y;
    obj_z = config.obj_z;
    obj_roll = config.obj_roll;
    obj_pitch = config.obj_pitch;
    obj_yaw = config.obj_yaw;
    eelink_x = config.eelink_x;
    eelink_y = config.eelink_y;
    eelink_z = config.eelink_z;
    eelink_roll = config.eelink_roll;
    eelink_pitch = config.eelink_pitch;
    eelink_yaw = config.eelink_yaw;
    grasp_angle = config.grasp_angle;

    enable_move_and_grasp_flag = config.enable_move_and_grasp_flag;
    enable_move_obj_flag = config.enable_move_obj_flag;
    reset_obj_pos_flag = config.reset_obj_pos_flag;
    enable_moveit_flag = config.enable_moveit_flag;
    enable_grasp_flag = config.enable_grasp_flag;
    save_image_flag = config.save_image_flag;
    save_mask_flag = config.save_mask_flag;
}
