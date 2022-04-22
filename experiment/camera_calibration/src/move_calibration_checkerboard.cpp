#include <iostream>
#include <math.h>
#include <string>
#include <unistd.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Bool.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;

cv::Mat RGB_img, IR_img, Depth_img;
bool RGB_update_flag = 0, IR_update_flag = 0, Depth_update_flag = 0;

void ir_Callback(const sensor_msgs::ImageConstPtr &msg)
{
    IR_update_flag = 0;
    try
    {
        IR_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        IR_update_flag = 1;
        /* 可选择是否显示图片 */
        // cv::imshow("ir_img", IR_img);
        // cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("ir Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
void rgb_Callback(const sensor_msgs::ImageConstPtr &msg)
{
    RGB_update_flag = 0;
    try
    {
        RGB_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        RGB_update_flag = 1;
        /* 可选择是否显示图片 */
        // cv::imshow("rgb_img", RGB_img);
        // cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("rgb Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void depth_Callback(const sensor_msgs::ImageConstPtr &msg)
{
    Depth_update_flag = 1;
    try
    {
        cv::Mat Depth_img_32FC1 = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone(); //得到的是32FC1的图片 对应type为5
        Depth_img_32FC1.convertTo(Depth_img, CV_16U, 65535.0 / 1.0, 0.0);
        Depth_update_flag = 1;
        /* 可选择是否显示图片 */
        // cv::imshow("Depth_img", Depth_img);
        // cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("depth Could not convert from depth_registered_Callback.");
    }
}

const string ir_topic = "/camera/ir/image_raw";
const string rgb_topic = "/camera/rgb/image_raw";
const string depth_topic = "/camera/depth/image_raw";
const char *save_dir = "./src/robot_sim/experiment/camera_calibration/save_checkboard_img/%s/%d.png";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_gazebo_model");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber ir_sub = it.subscribe(ir_topic, 1, ir_Callback);
    image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, rgb_Callback);
    image_transport::Subscriber depth_sub = it.subscribe(depth_topic, 1, depth_Callback);

    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState set_model_state_srv;
    gazebo_msgs::ModelState des_model_state;
    geometry_msgs::Twist twist;
    int ans;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    geometry_msgs::Pose pose;
    geometry_msgs::Quaternion quat;
    double x_bias = -0.04;
    double y_bias = -0.04;
    double z_bias = 0.2;
    double x, y, z;
    double dx = 0.3;
    double dy = 0.2;
    double dz = 0.25;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = 0.0;
    quat.w = 1.0;
    pose.orientation = quat;

    des_model_state.model_name = "small_checkerboard";
    des_model_state.pose = pose;
    des_model_state.twist = twist;
    des_model_state.reference_frame = "world";
    double qx, qy, qz, qw;

    // char *save_dir = getcwd(NULL, 0);
    // cout << save_dir << endl;

    while (ros::ok())
    {
        qx = 0.25 * (((rand() % 100) / 100.0) - 0.5);
        qy = 0.25 * (((rand() % 100) / 100.0) - 0.5);
        qz = 0.25 * (((rand() % 100) / 100.0) - 0.5);
        qw = 0.5;

        x = x_bias + dx * ((rand() % 100) / 100.0 - 0.5);
        y = y_bias + dy * ((rand() % 100) / 100.0 - 0.5);
        z = z_bias + dz * ((rand() % 100) / 100.0 - 0.5);

        double norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        quat.x = qx / norm;
        quat.y = qy / norm;
        quat.z = qz / norm;
        quat.w = qw / norm;

        cout << "qx, qy, qz, qw= " << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << endl;
        cout << "x,y,z = " << x << ", " << y << ", " << z << endl;
        pose.orientation = quat;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        des_model_state.pose = pose;
        set_model_state_srv.request.model_state = des_model_state;
        client.call(set_model_state_srv);

        ros::Duration(1).sleep();
        ros::spinOnce();

        if (RGB_update_flag && IR_update_flag && Depth_update_flag)
        {
            RGB_update_flag = 0;
            IR_update_flag = 0;
            Depth_update_flag = 0;

            cv::Size patternsize(7, 6); //标定板的角点个数
            vector<cv::Point2f> corners;
            bool RGB_OK = findChessboardCorners(RGB_img, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
            bool IR_OK = findChessboardCorners(IR_img, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
            if (RGB_OK && IR_OK)
            {
                IR_OK = 0;
                RGB_OK = 0;
                static int image_index = 1;
                char text[100];
                sprintf(text, save_dir, "RGB", image_index);
                cout << text << endl;
                imwrite(text, RGB_img);
                sprintf(text, save_dir, "Depth", image_index);
                imwrite(text, Depth_img);
                sprintf(text, save_dir, "IR", image_index);
                imwrite(text, IR_img);
                ROS_INFO("save %d done \n", image_index);
                image_index++;
            }
        }
    }
    return 0;
}
