#include <iostream>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
using namespace std;

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

double get_depth(uint16_t u_rgb, uint16_t v_rgb, cv::Mat &Depth_img)
{
    return Depth_img.at<uint16_t>(u_rgb, v_rgb) / 65535.0;
}

/**
* @brief	鼠标回调函数
* @param	event：鼠标事件 详情见cv::MouseEventTypes
* @param	x,y：鼠标的位置
* @param	flags：鼠标事件标志位 详情见cv::MouseEventFlags
* @param	mat是传进来的用户自定义参数 是void*类型
* @note		用户自定义参数可以用C++的元组打包发过来
*			只要没有destroy这个窗口则这个事件会一直存在,只要点了鼠标就会调用回调函数
*/
void mouse_callback(int event, int x, int y, int flags, void *mat)
{
    if (event == CV_EVENT_LBUTTONDOWN) //判断是否是鼠标左键按下
    {
        cv::Mat img = (*(cv::Mat *)mat).clone(); //创建一个img为传入参数的clone 可使得在img上画图不影响原来的图
        cv::Point pt = cv::Point(x, y);
        char text[16];
        sprintf(text, "(%d,%d)", x, y);
        putText(img, text, pt, CV_FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 255), 1); //在图片上显示文字
        printf("x=%d\ty=%d\t", x, y);                                                         //打印各种信息
        if ((*(cv::Mat *)mat).type() == 2)                                                    //如果是深度图则打印深度
        {
            printf("depth=%lf\n", img.at<uint16_t>(pt) / 65535.0);
        }
        else
        {
            printf("b=%d\t", img.at<cv::Vec3b>(pt)[0]);
            printf("g=%d\t", img.at<cv::Vec3b>(pt)[1]);
            printf("r=%d\n", img.at<cv::Vec3b>(pt)[2]);
        }
        circle(img, pt, 2, cv::Scalar(0, 0, 255), 2, 8); //在鼠标点击的地方画个圆
        cv::imshow("show", img);
        cv::waitKey(1);
    }
}

const char RGB_dir[] = "../../camera_calibration/save_checkboard_img/RGB/1.png";
const char Depth_dir[] = "../../camera_calibration/save_checkboard_img/Depth/1.png";

int main()
{
    ifstream in("../scripts/Registration_matrix.txt");
    char *buffer = new char[1024];
    while (!in.eof())
    {
        in.read(buffer, 1024);
    }
    double mat_ir2rgb[8];
    sscanf(buffer, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", mat_ir2rgb, mat_ir2rgb + 1, mat_ir2rgb + 2, mat_ir2rgb + 3, mat_ir2rgb + 4, mat_ir2rgb + 5, mat_ir2rgb + 6, mat_ir2rgb + 7);

    in.close();

    cv::Mat RGB_img = imread(RGB_dir, cv::IMREAD_UNCHANGED);
    cv::Mat Depth_img = imread(Depth_dir, cv::IMREAD_UNCHANGED);
    cv::Mat img_8UC1;
    Depth_img.convertTo(img_8UC1, CV_8U, 255.0 / 65535.0, 0.0); //转成8UC1来addWeighted来显示 对应type为0
    cv::Mat result;
    cv::Mat Channels[3] = {img_8UC1, img_8UC1, img_8UC1};
    merge(Channels, 3, result); //merge成3通道的8UC3 对应type为16
    addWeighted(result, 0.5, RGB_img, 0.5, 0.0, result);
    cv::imshow("origin_result", result); //原始的深度图揉进彩色图中

    cv::Mat output(Depth_img.rows, Depth_img.cols, CV_16UC1, cv::Scalar(0)); //初始化配准的深度图
    Depth2RGB(Depth_img, output, mat_ir2rgb);                            //将输入的Depth_img根据转移矩阵配准成output
    // cv::imshow("output", output);                                        //配准后的深度图
    // cv::waitKey(0);
    /* 显示配准后的揉在一起的效果 */
    cv::Mat temp;
    output.convertTo(temp, CV_8U, 255.0 / 65535.0, 0.0); //转成8UC1来addWeighted 对应type为0
    cv::Mat registration_result;
    cv::Mat new_Channels[3] = {temp, temp, temp};
    merge(new_Channels, 3, registration_result); //merge成3通道的8UC3 对应type为16
    addWeighted(registration_result, 0.5, RGB_img, 0.5, 0.0, registration_result);
    cv::imshow("registration_result", registration_result);

    setMouseCallback("registration_result", mouse_callback, &registration_result);
    setMouseCallback("origin_result", mouse_callback, &result);
    setMouseCallback("output", mouse_callback, &output);
    cv::waitKey(0);
    return 0;
}
