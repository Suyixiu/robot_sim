#include "control.h"
#include "debug.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

class TransformSender
{
public:
	ros::NodeHandle node_;
	TransformSender(double x, double y, double z, double yaw, double pitch, double roll, ros::Time time, const std::string &frame_id, const std::string &child_frame_id)
	{
		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		transform_ = tf::StampedTransform(tf::Transform(q, tf::Vector3(x, y, z)), time, frame_id, child_frame_id);
	};
	TransformSender(double x, double y, double z, double qx, double qy, double qz, double qw, ros::Time time, const std::string &frame_id, const std::string &child_frame_id) : transform_(tf::Transform(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z)), time, frame_id, child_frame_id){};
	~TransformSender() {}

	tf::TransformBroadcaster broadcaster;

	void send(ros::Time time)
	{
		transform_.stamp_ = time;
		broadcaster.sendTransform(transform_);
	};

private:
	tf::StampedTransform transform_;
};

#define math_pi 3.14159265
#define max_velocity 2.0
#define max_acceleration 2.0

#define ball_layer 1
#define ball_R 0.4
#define jingxian 4
#define weixian 5
#define pi 3.14159265

string rgb_topic = "/camera/rgb/image_raw";
string depth_topic = "/camera/depth/image_raw";
string pc_topic = "/camera/depth/points";

cv::Mat RGB_img, Depth_img;
pcl::PointCloud<pcl::PointXYZRGBA> cloud;

bool RGB_update_flag = 0, Depth_update_flag = 0, cloud_update_flag = 0;

char RGB_dir[] = "./src/robot_sim/experiment/data_collection/RGB/%d.png";
char Depth_dir[] = "./src/robot_sim/experiment/data_collection/Depth/%d.png";
char Point_cloud_dir[] = "./src/robot_sim/experiment/data_collection/Point_cloud/%d.pcd";

/* 保存点云为pcd */
void cloudCB(const sensor_msgs::PointCloud2 &msg)
{
	pcl::fromROSMsg(msg, cloud); //从ROS类型消息转为PCL类型消息
	cloud_update_flag = 1;
}
/* 保存RGB图像 */
void rgb_Callback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		RGB_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
		RGB_update_flag = 1;
		// cv::imshow("rgb_img", RGB_img);
		// cv::waitKey(1);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("rgb Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}
/* 保存深度图像 */
void depth_Callback(const sensor_msgs::ImageConstPtr &msg)
{
	// cout << msg->encoding << endl;
	try
	{
		Depth_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image.clone();
		Depth_update_flag = 1;
		// cv::imshow("depth_img", Depth_img);
		// cv::waitKey(1);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("depth Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

string data_num_txt_dir = "./src/robot_sim/experiment/data_collection/index_num.txt";

int readWriteFile(string srcFile)
{
	// cout <<"in File" << srcFile << endl;
	ifstream in(srcFile.c_str());

	char *buffer = new char[1024];
	while (!in.eof())
	{
		in.read(buffer, 1024);
		// cout << buffer << endl;
	}

	int data_num = 0;
	sscanf(buffer, "%d", &data_num);

	in.close();
	return data_num;
}

int main(int argc, char **argv)
{
	static int bad_grasp_count = 1;
	ros::init(argc, argv, "collect_data");
	ros::NodeHandle nh;

	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, rgb_Callback);
	image_transport::Subscriber depth_sub = it.subscribe(depth_topic, 1, depth_Callback);

	ros::Subscriber bat_sub = nh.subscribe(pc_topic, 10, cloudCB);
	ROS_INFO("let's rock!\n");

	dynamic_reconfigure::Server<robot_sim::data_collectionConfig> server;
	dynamic_reconfigure::Server<robot_sim::data_collectionConfig>::CallbackType callback;

	callback = boost::bind(&data_collection_dynamic_callback, _1);
	server.setCallback(callback);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Rate loop_rate(1000); //设置发送数据的频率为1000Hz
	ros::Duration sleeper(100 / 1000.0);

	/* world到球面中心的tf */
	// tf::Transform world2center;
	// tf::Quaternion world2center_Q;
	// world2center_Q.setRPY(0, 0, 0);
	// world2center.setRotation(world2center_Q);
	// world2center.setOrigin(tf::Vector3(-0.15785, 0.63107, 0.6549));
	// TransformSender center_sender(-0.15785, 0.63107, 0.6549, 0, 0, 0, ros::Time() + sleeper, "world", "center");
	// center_sender.send(ros::Time::now() + sleeper);

	std::vector<tf::Transform> world2point_set;
	std::vector<TransformSender> tf_sender;
	/* 目标是得到球面坐标系上的n个点的xyz和rpy 从正右方开始 */
	/* 赤道上分4个经度 */
	for (int layer = 1; layer <= ball_layer; layer++)
	{
		for (int i = 0; i < jingxian; i++)
		{
			/* 纬度上 一根经线取7个点 其中3个的纬度一样 还有一个是定点 */
			for (int j = 0; j < weixian; j++)
			{
				double theta = math_pi * (90 / (weixian + 1)) * ((weixian - 1) / 2 - j) / 180;
				double phi = math_pi * (360 / jingxian / 2) * i / 180;
				double x = layer * ball_R * sin(theta) * cos(phi) - 0.13;
				double y = layer * ball_R * sin(theta) * sin(phi) + 0.75;
				double z = layer * ball_R * cos(theta) + 1.2;
				char point_label[100];
				sprintf(point_label, "p%d,%d,%d", layer, i, j);

				double roll = theta * sin(-phi);
				double pitch = theta * cos(phi);
				double yaw = 0;

				TransformSender tf_sender_temp(x, y, z, yaw, pitch, roll, ros::Time() + sleeper, "world", point_label);
				tf_sender.push_back(tf_sender_temp);

				/* world到球面的点tf */
				tf::Transform world2point;
				tf::Quaternion world2point_Q;
				world2point_Q.setRPY(roll, pitch, yaw);
				world2point.setRotation(world2point_Q);
				world2point.setOrigin(tf::Vector3(x + 0.01757, y - 0.128, z - 0.1425));
				world2point_set.push_back(world2point);
			}
		}
	}

	grasp(0.6);
	grasp(0.8);
	go_home();
	reset_obj_pos();
	/* 先把点打出来看看位置对不对先 */
	for (int i = 0; i < ball_layer * jingxian * weixian; i++)
	{
		tf_sender[i].send(ros::Time::now() + sleeper);
		sleeper.sleep();
	}

	int point_index = readWriteFile(data_num_txt_dir);
	int init_data_num = readWriteFile(data_num_txt_dir);
	int move_fail_time = 0;
	while (ros::ok()) //ros::ok()返回false会停止运行，进程终止。
	{
		/* 如果收到爪子且是好的则让机械臂过去 */
		if (next_pose)
		{
			// next_pose = 0;
			int tf_index = 0;
			if (init_data_num == 0)
			{
				init_data_num = 20;
				tf_index = 0;
			}
			else
			{
				tf_index = point_index % 20;
			}

			if (line_move(world2point_set[tf_index], 0.1, 0.1))
			{
				std::cout << point_index << endl;

				ros::Duration(0.5).sleep();
				ros::Duration(2).sleep();
				char save_dir[100];
				sprintf(save_dir, RGB_dir, point_index);
				imwrite(save_dir, RGB_img);

				sprintf(save_dir, Depth_dir, point_index);
				imwrite(save_dir, Depth_img);

				sprintf(save_dir, Point_cloud_dir, point_index);
				pcl::io::savePCDFileASCII(save_dir, cloud); //保存原始的pcd

				cout << "save done!" << endl;

				point_index += 1;
				// if (point_index % (ball_layer * jingxian * weixian) == 0)
				// 	point_index += init_data_num;
			}
			else
			{
				move_fail_time += 1;
				cout << "fail " << move_fail_time << endl;
				if (move_fail_time == 3)
				{
					move_fail_time = 0;
					ros::Duration(0.5).sleep();
					char save_dir[100];
					sprintf(save_dir, RGB_dir, point_index);
					imwrite(save_dir, RGB_img);

					sprintf(save_dir, Depth_dir, point_index);
					imwrite(save_dir, Depth_img);

					sprintf(save_dir, Point_cloud_dir, point_index);
					pcl::io::savePCDFileASCII(save_dir, cloud); //保存原始的pcd

					cout << "save done!" << endl;

					point_index += 1;
					// if (point_index % (ball_layer * jingxian * weixian) == 0)
					// 	point_index += init_data_num;
				}
			}
		}
		if (save_data && RGB_update_flag && Depth_update_flag && cloud_update_flag)
		{
			save_data = 0;
			RGB_update_flag = 0;
			Depth_update_flag = 0;
			cloud_update_flag = 0;

			char save_dir[100];
			sprintf(save_dir, RGB_dir, point_index);
			imwrite(save_dir, RGB_img);

			sprintf(save_dir, Depth_dir, point_index);
			imwrite(save_dir, Depth_img);

			sprintf(save_dir, Point_cloud_dir, point_index);
			pcl::io::savePCDFileASCII(save_dir, cloud); //保存原始的pcd

			cout << "save done!" << endl;
			// go_home();
		}
		ros::spinOnce();   //不是必须，若程序中订阅话题则必须，否则回掉函数不起作用。
		loop_rate.sleep(); //按前面设置的10Hz频率将程序挂起
	}

	return 0;
}