#ifndef _RECOGNIZE_H_
#define _RECOGNIZE_H_

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"

using namespace std;

extern cv::Mat RGB_img, IR_img, Depth_img;
extern bool RGB_update_flag, IR_update_flag, Depth_update_flag;
extern cv::Point target_pos;  //轮廓的中心
extern double main_direction; //图形的主方向
extern double grasp_distance; //两个抓取点的距离

void rgb_Callback(const sensor_msgs::ImageConstPtr &msg);
void depth_Callback(const sensor_msgs::ImageConstPtr &msg);

void recognize_init();
cv::Point3f recognize(cv::Mat RGB_mask);
void my_addWeighted(cv::Mat &src, cv::Mat &add_img, const char *window_name);

#endif