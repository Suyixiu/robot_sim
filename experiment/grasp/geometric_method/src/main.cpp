#include "control.h"
#include "debug.h"
#include "recognize.h"

const string rgb_topic = "/camera/rgb/image_raw";
const string depth_topic = "/camera/depth/image_raw";

string RGBmask_dir = "./src/robot_sim/experiment/grasp/img/RGB_mask.png";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_sim");
    ROS_INFO("use geometric method to grasp!\n");
    ros::NodeHandle nh;
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, rgb_Callback);
    image_transport::Subscriber depth_sub = it.subscribe(depth_topic, 1, depth_Callback);

    dynamic_reconfigure::Server<robot_sim::graspConfig> server;
    dynamic_reconfigure::Server<robot_sim::graspConfig>::CallbackType callback;

    callback = boost::bind(&grasp_dynamic_callback, _1);
    server.setCallback(callback);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loop_rate(1000); //设置发送数据的频率为1000Hz

    go_home();
    grasp(0.6);
    grasp(0.8);
    reset_obj_pos();

    cv::Mat RGB_mask = imread(RGBmask_dir, cv::IMREAD_UNCHANGED);

    recognize_init(); //创建滑条

    while (ros::ok()) //ros::ok()返回false会停止运行，进程终止。
    {
        if (RGB_update_flag && Depth_update_flag)
        {
            RGB_update_flag = 0;
            Depth_update_flag = 0;
            imshow("1 rgb_img", RGB_img);
            imshow("2 depth_img", Depth_img);
            cv::Point3f target_pos = recognize(RGB_mask);
            cv::waitKey(1);
            if (save_image_flag)
            {
                save_image_flag = false;
                static int image_index = 1;
                char text[20];
                sprintf(text, "./src/robot_sim/experiment/grasp/img/RGB_%d.png", image_index);
                imwrite(text, RGB_img);
                sprintf(text, "./src/robot_sim/experiment/grasp/img/Depth_%d.png", image_index);
                imwrite(text, Depth_img);
                ROS_INFO("save %d done \n", image_index);
                image_index++;
            }
            if (enable_move_obj_flag)
            {
                enable_move_obj_flag = false;
                move_obj(obj_name, obj_x, obj_y, obj_z, obj_roll, obj_pitch, obj_yaw);
                ROS_INFO("move obj done!\n");
            }
            if (reset_obj_pos_flag)
            {
                reset_obj_pos_flag = false;
                reset_obj_pos();
                // go_home();
                ROS_INFO("rest obj pos done!\n");
            }
            if (enable_move_and_grasp_flag)
            {
                enable_move_and_grasp_flag = false;
                if (target_pos.z < -0.19)
                    target_pos.z = -0.19;
                target_pos.z = target_pos.z + 1.2;
                double gripper_angle = 0.0033113173071797 * grasp_distance + 0.127363015998244; //设定的值
                ROS_INFO("x:%lf\ty:%lf\tz:%lf\tmain_direction:%lf\tgrasp_angle:%lf\tgrasp_distance:%lf\n", target_pos.x, target_pos.y, target_pos.z, main_direction, gripper_angle, grasp_distance);
                move(target_pos.x, target_pos.y, target_pos.z + 0.1, eelink_roll, eelink_pitch, main_direction);
                move(target_pos.x, target_pos.y, target_pos.z, eelink_roll, eelink_pitch, main_direction);
                grasp(0.33);
                go_home();
                move(-0.465, 0.24, 1.4, 0.0, 0.0, 0.0); //箱子上边
                grasp(0.8);
                go_home();
            }
            if (enable_moveit_flag)
            {
                enable_moveit_flag = false;
                move(eelink_x, eelink_y, eelink_z, eelink_roll, eelink_pitch, eelink_yaw);
                ROS_INFO("moveit done!\n");
            }
            if (enable_grasp_flag)
            {
                enable_grasp_flag = false;
                grasp(grasp_angle);
                ROS_INFO("grasp done!\n");
            }
        }
        ros::spinOnce();   //不是必须，若程序中订阅话题则必须，否则回掉函数不起作用。
        loop_rate.sleep(); //按前面设置的10Hz频率将程序挂起
    }
    return 0;
}