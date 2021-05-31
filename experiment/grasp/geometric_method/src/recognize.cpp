#include "recognize.h"

#define math_pi 3.1415926

Mat RGB_img, IR_img, Depth_img;
bool RGB_update_flag = 0, IR_update_flag = 0, Depth_update_flag = 0;

void rgb_Callback(const sensor_msgs::ImageConstPtr &msg)
{
	RGB_update_flag = 0;
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

void depth_Callback(const sensor_msgs::ImageConstPtr &msg)
{
	Depth_update_flag = 0;
	try
	{
		Mat Depth_img_32FC1 = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone(); //得到的是32FC1的图片 对应type为5
		Depth_img_32FC1.convertTo(Depth_img, CV_16U, 65535.0 / 1.0, 0.0);
		Depth_update_flag = 1;
		// cv::imshow("depth_img", Depth_img);
		// Mat img_8UC1;
		// Depth_img.convertTo(img_8UC1, CV_8U, 255.0 / 65535, 0.0); //转成8UC1来addWeighted 对应type为0
		// Mat result;
		// Mat Channels[3] = {img_8UC1, img_8UC1, img_8UC1};
		// merge(Channels, 3, result); //merge成3通道的8UC3 对应type为16
		// addWeighted(result, 0.5, RGB_img, 0.5, 0.0, result);
		// cv::imshow("depth_result_img", result);
		// cv::waitKey(1);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("depth Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void bar_callback(int, void *mat)
{
}

/* 深度图配准到RGB图的矩阵 */
// double mat_ir2rgb[] = {1.000070, -0.000001, -0.099105, -29.447156, 0.000109, 1.000041, -0.104798, 0.010875}; //乘上ir的uv索引出rgb的uv
double mat_ir2rgb[] = {1.000024, -0.000215, 0.008221, -17.652585, 0.000066, 0.999908, 0.345271, -0.007065};


/**
* @brief	深度图配准到RGB函数
* @param	Depth_img：原始深度图
* @param	dst：经过配准之后的深度图
* @param	w：深度图配准到RGB图的矩阵
* @note     原理是遍历深度图的每一个坐标然后通过配准矩阵映射到RGB中
*           然后把深度赋值过去 此外在映射失败像素点则用能重新映射上的点填上
*/
void Depth2RGB(cv::Mat &Depth_img, cv::Mat &dst, const double *W)
{
	double z;
	uint16_t u_ir, v_ir, d;
	uint16_t u_rgb, v_rgb;
	uint16_t u_rgb_old = 0;
	cv::Mat newdepth(Depth_img.rows, Depth_img.cols, CV_16UC1, cv::Scalar(0));
	// cv::Mat newdepth(dst.rows, dst.cols, CV_16UC1, cv::Scalar(0));
	static int count = 1;
	for (v_ir = 0; v_ir < Depth_img.rows; v_ir++)
	{
		for (u_ir = 0; u_ir < Depth_img.cols; u_ir++)
		{
			int type = Depth_img.type();
			d = Depth_img.at<uint16_t>(v_ir, u_ir);
			z = (double)d / 65535.0;
			u_rgb = (uint16_t)((W[0] * (double)u_ir + W[1] * (double)v_ir + W[2] + W[3] / z));
			v_rgb = (uint16_t)((W[4] * (double)u_ir + W[5] * (double)v_ir + W[6] + W[7] / z));

			if (u_rgb >= 0 && u_rgb <= newdepth.cols && v_rgb >= 0 && v_rgb <= newdepth.rows)
			{
				int delta = u_rgb - u_rgb_old - 1;
				if (delta > 0)
				{
					for (int i = 0; i < delta + 1; i++)
					{
						uint16_t *val = (uint16_t *)newdepth.ptr<uchar>(v_rgb) + u_rgb_old + i + 1;
						*val = d;
					}
					u_rgb_old = u_rgb;
				}
				else
				{
					uint16_t *val = (uint16_t *)newdepth.ptr<uchar>(v_rgb) + u_rgb;
					*val = d;
					u_rgb_old = u_rgb;
				}
			}
		}
	}
	dst = newdepth;
}

/**
* @brief	获取深度函数
* @param	u_rgb：深度图的横坐标
* @param	v_rgb：深度图的纵坐标
* @param	Depth_img：要获取深度的深度图
*/
double get_depth(uint16_t u_rgb, uint16_t v_rgb, cv::Mat &Depth_img)
{
	return Depth_img.at<uint16_t>(u_rgb, v_rgb) / 65535.0;
}

/* 手眼标定得到的转移关系 是眼坐标到手坐标的关系 */
double qw = 0;
double qx = 1;
double qy = 0;
double qz = 0;
double tx = 0.0175;
double ty = 0.128;
double tz = 0.1425;

Mat hand_eye_quaternion = (cv::Mat_<float>(4, 1) << qw, qx, qy, qz);

/**
* @brief	四元数转成旋转矩阵函数
* @param	q：四元数
*/
Mat Quaternion2Matrix(cv::Mat q)
{
	float w = q.at<float>(0);
	float x = q.at<float>(1);
	float y = q.at<float>(2);
	float z = q.at<float>(3);

	float xx = x * x;
	float yy = y * y;
	float zz = z * z;
	float xy = x * y;
	float wz = w * z;
	float wy = w * y;
	float xz = x * z;
	float yz = y * z;
	float wx = w * x;

	float ret[4][4];
	ret[0][0] = 1.0f - 2 * (yy + zz);
	ret[0][1] = 2 * (xy - wz);
	ret[0][2] = 2 * (wy + xz);
	ret[0][3] = 0.0f;

	ret[1][0] = 2 * (xy + wz);
	ret[1][1] = 1.0f - 2 * (xx + zz);
	ret[1][2] = 2 * (yz - wx);
	ret[1][3] = 0.0f;

	ret[2][0] = 2 * (xz - wy);
	ret[2][1] = 2 * (yz + wx);
	ret[2][2] = 1.0f - 2 * (xx + yy);
	ret[2][3] = 0.0f;

	ret[3][0] = 0.0f;
	ret[3][1] = 0.0f;
	ret[3][2] = 0.0f;
	ret[3][3] = 1.0f;

	return cv::Mat(4, 4, CV_32FC1, ret).clone();
}

/**
* @brief	获取相机坐标系下物体位置函数
* @param	picture_frame_pos：图像坐标系中的坐标点
* @param	z：该点的深度
* @note     得到的是4x1的矩阵
*/
Mat get_camra_frame_pos(Point picture_frame_pos, double z)
{
	double fx, fy, cx, cy, k1, k2, k3, p1, p2;
	fx = 1178.7329083650782;
	fy = 1178.7329083650782;
	cx = 640;
	cy = 360;
	k1 = 0;
	k2 = 0;
	k3 = 0;
	p1 = 0;
	p2 = 0;

	Mat cameraMatrix = (cv::Mat_<float>(3, 3) << fx, 0.0, cx,
						0.0, fy, cy,
						0.0, 0.0, 1.0);
	Mat distCoeffs = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);

	Mat input_pos = (cv::Mat_<float>(3, 1) << picture_frame_pos.x, picture_frame_pos.y, 1.0);

	Mat camra_frame_pos_3x1 = cameraMatrix.inv() * z * input_pos;

	Mat camra_frame_pos_4x1 = (cv::Mat_<float>(4, 1) << camra_frame_pos_3x1.at<float>(0, 0),
							   camra_frame_pos_3x1.at<float>(0, 1),
							   camra_frame_pos_3x1.at<float>(0, 2),
							   1.0);
	return camra_frame_pos_4x1;
}

void center_g(const vector<Point> contour, Point &center)
{
	Moments mu;
	mu = moments(contour, false);
	center.x = mu.m10 / mu.m00;
	center.y = mu.m01 / mu.m00;
}

float distance(Point p1, Point p2)
{
	float distance = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
	return distance;
}

void get_grasp_point(vector<Point> &pts, Point &center, double *main_direction, double *grasp_distance, Mat &to_draw)
{
	Mat data_pts = Mat(pts.size(), 2, CV_64FC1); //使用mat来保存数据，也是为了后面pca处理需要
	for (int i = 0; i < data_pts.rows; ++i)
	{
		data_pts.at<double>(i, 0) = pts[i].x;
		data_pts.at<double>(i, 1) = pts[i].y;
	}
	/* PCA */
	PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
	// center = Point(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1)); //均值就是中心
	center_g(pts, center);
	vector<Point2d> eigen_vecs(2); //特征向量
	vector<double> eigen_val(2);   //特征值
	for (int i = 0; i < 2; ++i)
	{
		eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
		eigen_val[i] = pca_analysis.eigenvalues.at<double>(i, 0);
	}
	/* 绘制中心 主轴 副轴 */
	circle(to_draw, center, 3, CV_RGB(0, 255, 0), -1);
	// line(to_draw, center, center + 0.02 * Point(eigen_vecs[0].x * 3000, eigen_vecs[0].y * 3000), CV_RGB(255, 0, 0));
	// line(to_draw, center, center + 0.02 * Point(eigen_vecs[1].x * 3000, eigen_vecs[1].y * 3000), CV_RGB(0, 0, 255));

	/* 角度结果 */
	*main_direction = atan2(eigen_vecs[0].y, eigen_vecs[0].x) * 180 / math_pi; //主轴
	/* 绘制抓取的矩形框 ICIG */
	RotatedRect rRect = RotatedRect(center, cv::Size(50, 150), *main_direction);
	Point2f vertices[4];
	rRect.points(vertices);
	for (int i = 0; i < 4; i++)
		line(to_draw, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 3);
	// Rect brect = rRect.boundingRect();
	// rectangle(to_draw, brect, Scalar(255, 0, 0));

	// *main_direction = atan2(eigen_vecs[1].y, eigen_vecs[1].x) * 180 / math_pi; //副轴
	*main_direction = -90 - *main_direction;
	// cout << *main_direction << endl;

	/* 获得轮廓最小外接矩形 */
	Rect boundRect = boundingRect(Mat(pts));
	// cv::rectangle(to_draw,boundRect,Scalar(0,0,255));
	// imshow("min_rect", to_draw);
	// waitKey(0);

	/* 计算过垂直主方向直线与轮廓的两个交点 */
	// float delta2 = tan((main_direction + 90) * math_pi / 180); //斜率
	float delat = eigen_vecs[1].y / eigen_vecs[1].x; //斜率
	Point2f line_end_1 = Point2f((boundRect.y - center.y) / delat + center.x, boundRect.y);
	Point2f line_end_2 = Point2f((boundRect.y + boundRect.height - center.y) / delat + center.x, boundRect.y + boundRect.height);
	/* 遍历两个交点之间的线段,垂直轴方向,得出和轮廓的交点 */
	LineIterator it_sub(to_draw, line_end_1, line_end_2, 8);
	vector<Point> grasp_point;
	vector<int> best_point_index;
	for (int i = 0; i < it_sub.count; i++, ++it_sub) //点是否在轮廓上
	{
		Point pt(it_sub.pos()); //获得线段上的点
		// int index = 0;
		// if (abs(my_pointPolygonTest(pts, pt, true, &index)) < 1) //检测点是否在轮廓上
		if (abs(pointPolygonTest(pts, pt, true)) < 1) //检测点是否在轮廓上
		{
			grasp_point.push_back(pt);
			// circle(to_draw, pt, 5, Scalar(0, 0, 255), -1);
			
			// best_point_index.push_back(index);
			// circle(img_show1, pt, 5, Scalar(0, 0, 255), -1); //若是则画上
			// circle(img_show1, pts[index + 10], 5, Scalar(255, 0, 0), -1); //逆时针10个点
			// circle(img_show1, pts[index-10], 5, Scalar(255, 0, 0), -1);//顺时针10个点
		}
	}
	*grasp_distance = distance(grasp_point[0], grasp_point[grasp_point.size() - 1]);
	// cout << *main_direction << "\t" << *grasp_distance << endl;
}

Point target_pos;									  //轮廓的中心
double main_direction;								  //图形的主方向
double grasp_distance;								  //两个抓取点的距离
int CannySize = 30, DilateSize = 4, GaussianSize = 3; //滑条参数用于图像处理阈值调整

void recognize_init()
{
	namedWindow("bar_window");		  //创建一个窗口bar_window
	cvMoveWindow("bar_window", 0, 0); //移动这个窗口

	createTrackbar("GaussianSize", "bar_window", &GaussianSize, 255, bar_callback);
	createTrackbar("CannySize", "bar_window", &CannySize, 255, bar_callback);
	createTrackbar("penzhang", "bar_window", &DilateSize, 255, bar_callback);
}

Point3f recognize(Mat RGB_mask)
{
	Mat registration_depth(Depth_img.rows, Depth_img.cols, CV_16UC1, cv::Scalar(0)); //初始化配准的深度图
	Depth2RGB(Depth_img, registration_depth, mat_ir2rgb);							 //将输入的Depth_img根据转移矩阵配准成registration_depth
	cv::imshow("registration_depth", registration_depth);							 //配准后的深度图

	/* 显示配准后的揉在一起的效果 */
	// Mat temp;
	// registration_depth.convertTo(temp, CV_8U, 255.0 / 1000, 0.0); //转成8UC1来addWeighted 对应type为0
	// Mat registration_result;
	// Mat new_Channels[3] = {temp, temp, temp};
	// merge(new_Channels, 3, registration_result); //merge成3通道的8UC3 对应type为16
	// addWeighted(registration_result, 0.5, RGB_img, 0.5, 0.0, registration_result);

	// imwrite("/home/suyixiu/catkin_ws/src/yixiuge_ur/data/RGB_mask.png", RGB_img);
	// cv::imshow("registration_result", registration_result);

	Mat RGB_diff, RGB_diff_gray, RGB_diff_bin, RGB_diff_canny;
	RGB_diff = abs(RGB_img - RGB_mask);
	imshow("3 RGB_diff", RGB_diff);
	GaussianBlur(RGB_diff, RGB_diff, Size(GaussianSize * 2 + 1, GaussianSize * 2 + 1), 3, 3);
	cvtColor(RGB_diff, RGB_diff_gray, CV_BGR2GRAY); //转成灰度图
	threshold(RGB_diff_gray, RGB_diff_bin, 5, 255, THRESH_BINARY); //设阈值得到二值图
	imshow("4 RGB_diff_bin", RGB_diff_bin);

	Mat img_show1 = RGB_img.clone();
	Mat img_show2 = RGB_img.clone();
	Mat img_show3 = RGB_img.clone();

	Canny(RGB_diff_gray, RGB_diff_canny, CannySize, 3 * CannySize, 3); //第三个参数是第一个滞后性阈值 第四个参数是第二个滞后性阈值 推荐的高低阈值比在2:1到3:1之间 第五个是Sobel核的大小默认为3
	imshow("4 canny", RGB_diff_canny);

	Mat element = getStructuringElement(MORPH_RECT, Size(DilateSize, DilateSize)); //自定义的核 第一个参数是核的形状 这里用的是矩形 详情见MorphShapes
	dilate(RGB_diff_canny, RGB_diff_canny, element);							   //膨胀 也就是求局部最大值 亮的地方更亮
	imshow("5 dilate_result", RGB_diff_canny);

	erode(RGB_diff_canny, RGB_diff_canny, element); //腐蚀 也就是求局部最小值 暗的地方更暗
	imshow("6 erode_result", RGB_diff_canny);

	vector<vector<Point>> contours;
	findContours(RGB_diff_bin, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	/* 选出最大轮廓 */
	size_t biggest_contours_size = 0;
	vector<Point> biggest_contours;
	int biggest_index = 0;
	for (size_t i = 0; i < contours.size(); i++)
	{
		if (biggest_contours_size < contours[i].size())
		{
			biggest_contours_size = contours[i].size();
			biggest_contours = contours[i];
			biggest_index = i;
		}
	}

	/* 识别到然后得到轮廓了 */
	if (biggest_contours.size() > 0)
	{
		get_grasp_point(biggest_contours, target_pos, &main_direction, &grasp_distance, img_show1);
		// center_g(biggest_contours, target_pos);
		drawContours(img_show1, contours, biggest_index, Scalar(255, 0, 255), 2);
		// circle(img_show1, target_pos, 3, Scalar(0, 255, 0), -1); //绘制圆心
		imshow("7 The biggest contour and final result", img_show1);

		double target_pos_height = get_depth(target_pos.y, target_pos.x, registration_depth);

		// cout << target_pos_height << endl;
		// cout << target_pos << endl;

		Mat camra_frame_pos_4x1 = get_camra_frame_pos(target_pos, target_pos_height);

		// cout << camra_frame_pos_4x1 << endl;

		Mat R = Quaternion2Matrix(hand_eye_quaternion);

		Mat T = (cv::Mat_<float>(4, 4) << 1.0, 0.0, 0.0, -tx,
				 0.0, 1.0, 0.0, -ty,
				 0.0, 0.0, 1.0, -tz,
				 0.0, 0.0, 0.0, 1.0);

		Mat gripper_frame_pos_4x1 = R * T * camra_frame_pos_4x1;

		/* world_frame_pos_3x1是最终结果 因为每次都是回到home再去抓所以没有旋转 就很奈斯 */
		Mat world_frame_pos_3x1 = (cv::Mat_<float>(3, 1) << gripper_frame_pos_4x1.at<float>(0, 0) - 0.16385,
								   gripper_frame_pos_4x1.at<float>(0, 1) + 0.67312,
								   gripper_frame_pos_4x1.at<float>(0, 2) + 0.3355);

		Point3f result(world_frame_pos_3x1.at<float>(0, 0), world_frame_pos_3x1.at<float>(0, 1), world_frame_pos_3x1.at<float>(0, 2));
		return result;
	}
	else
	{
		return Point3f(-0.11, 0.67, 0.1); //没识别到就呆在home
	}
}