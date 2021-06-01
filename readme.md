# A ROS based Open Source Simulation Environment for Robotics Beginners
基于ROS搭建的机器人仿真环境。其中设计了机器人研究过程中需要预先进行的几个重要实验如各种标定(相机标定，深度图配准，手眼标定)，还在仿真环境中实现了两种抓取算法的仿真，一种是传统方法(几何方法)一种是机器学习的方法(GPD)，最后一个实验是模拟机械臂采集数据的场景。  
视频链接-->[Bilibili](https://www.bilibili.com/video/BV19f4y1h73E)  

| Author | 苏一休            |
| ------ | ----------------- |
| E-mail | 2270678755@qq.com |

## 目录
* [搭建](#搭建)
* [在仿真环境中进行相机标定](#在仿真环境中进行相机标定)
* [在仿真环境中进行深度图配准](#在仿真环境中进行深度图配准)
* [在仿真环境中进行手眼标定](#在仿真环境中进行手眼标定)  
* [在仿真环境中抓取](#在仿真环境中抓取)  
    * [基于传统方法](#基于传统方法)
    * [基于机器学习方法](#基于机器学习方法)
* [在仿真环境中进行数据采集](#在仿真环境中进行数据采集)  
* [总结](#总结)

## 搭建

## 在仿真环境中进行相机标定
首先是将相机模型还有标定板模型load进来
```
roslaunch robot_sim camera_calibration.launch
```
之后便可以看到RealSense D435i RGBD相机与标定板，其中这里使用的是7x6内角点，块块大小0.01m的标定板。若想使用其他尺寸的标定板可更改`experiment/camera_calibration/urdf/create_chessboard.py`脚本中的标定板参数后运行以在`experiment/camera_calibration/urdf`目录下生成你所需要的标定板，随后修改`camera_calibration.launch`要load的标定板即可。
<img src="https://z3.ax1x.com/2021/06/01/2uCb9A.jpg" width = "600" />  
<!-- <center><img src="https://z3.ax1x.com/2021/06/01/2uCb9A.jpg" width = "700" /></center>   -->

其中相机的URDF文件中使用的相机插件设置的相机视角是$57\degree$，图像分辨率是1280x720，所以根据相机内参各个参数的的定义算出该相机模型的内参真实值。这个计算与`camera_info`这个topic中的信息一致。
$$
\begin{aligned}
f_x=f_y&=\frac{image\_width / 2}{tan(fov / 2)} = \frac{1280 / 2}{tan(57 / 2)} = 1178.73\\
c_x &= image\_width / 2 = 1280 / 2 = 640\\
c_y &= image\_height / 2 = 720 / 2 =360
\end{aligned}
$$

此时相机已以30帧往外发布图像信息。可以使用ROS自带的标定包来进行实时的标定，也可以把图片保存下来后用相机标定工具如Matlab中的相机标定包进行内参的计算。这里以ROS功能包camera_calibration中的`cameracalibrator.py`脚本为例。你可以选择你所需要标定的相机。
```
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.01 image:=/camera/rgb/image_raw camera:=/camera/rgb     #RGB相机
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.01 image:=/camera/ir/image_raw camera:=/camera/ir       #IR相机
```
<img src="https://z3.ax1x.com/2021/06/01/2uAIW4.png" width = "600" />  

接下来是移动标定板，程序中设定的是1秒钟产生一个随机位置并移动标定板，若相机能正确识别出标定板角点则将此时的照片保存，有深度图，红外相机的图还有RGB图存放在`save_checkboard_img`目录中。当你觉得采的图片足够多之后即可停止这个程序。
```
rosrun robot_sim camera_calibration
```
这里我们提供了一个python的脚本`camera_calibration.py`用于载入前面保存的图片以计算RGB相机与IR相机的内参并分别保存在`IR_cameraintrinsic_parameters.npz`与`RGB_cameraintrinsic_parameters.npz`中，直接python运行此脚本即可。你也可以自己写程序来计算内参，并与前面公式计算的结果像对比以验证你的标定算法的准确性与误差。
```
python3 camera_calibration.py
```

## ## 在仿真环境中进行深度图配准
此实验将用到实验一中采集的两个相机拍摄的标定板的图片

## 实验三：hand_eye_calibration
```
roslaunch yixiuge_ur10_moveit_config yixiuge_ur_moveit.launch
```
需要等moveit加载完之后再加载hand_eye
```
roslaunch robot_sim hand_eye_calibration.launch
```

## 实验四：grasp
这里设计了两种方法的抓取，一种是使用传统的几何方法识别抓取点的抓取，一种是基于机器学习的抓取
### 基于几何方法的抓取
```
roslaunch robot_sim geometric_method_grasp.launch 
rosrun robot_sim geometric_method_grasp
```

### 基于机器学习方法的抓取
```
roslaunch robot_sim GPD_method_grasp.launch
roslaunch robot_sim gpd_run.launch type:=2 topic:=/cloud_sample
```

## 实验五：data_collection
```
```
