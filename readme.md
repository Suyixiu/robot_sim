# A ROS based Open Source Simulation Environment for Robotics Beginners
基于ROS搭建的机器人仿真环境。其中设计了机器人研究过程中需要预先进行的几个重要实验如各种标定(相机标定，深度图配准，手眼标定)，还在仿真环境中实现了两种抓取算法的仿真，一种是传统方法(几何方法)一种是机器学习的方法(GPD)，最后一个实验是模拟机械臂采集数据的场景。  
视频链接-->[Bilibili](https://www.bilibili.com/video/BV19f4y1h73E)&[YouTube](https://youtu.be/Ky5vFTKUd1w)  

| Author | 苏一休            |
| ------ | ----------------- |
| E-mail | 2270678755@qq.com |

## 目录
- [A ROS based Open Source Simulation Environment for Robotics Beginners](#a-ros-based-open-source-simulation-environment-for-robotics-beginners)
  - [目录](#目录)
  - [搭建](#搭建)
  - [在仿真环境中进行相机标定](#在仿真环境中进行相机标定)
  - [在仿真环境中进行深度图配准](#在仿真环境中进行深度图配准)
  - [在仿真环境中进行手眼标定](#在仿真环境中进行手眼标定)
  - [在仿真环境中抓取](#在仿真环境中抓取)
    - [基于几何方法的抓取](#基于几何方法的抓取)
    - [基于机器学习方法的抓取](#基于机器学习方法的抓取)
  - [在仿真环境中进行数据采集](#在仿真环境中进行数据采集)
  - [总结](#总结)

## 搭建  
此功能包在**Ubuntu16.04**上经过测试，应该适用于其他Linux版本。在你的Catkin工作空间中需要有
* 负责计算aruco二维码位姿态的[aruco_ros](https://github.com/pal-robotics/aruco_ros/tree/kinetic-devel)；
* 使用点云的基于深度学习的抓取位姿检测[gpd_ros](https://github.com/atenpas/gpd_ros/)(这个包还需要编译安装[GPD library](https://github.com/atenpas/gpd))；
* 手眼标定功能包[easy_handeye](https://github.com/IFL-CAMP/easy_handeye)；
* UR机械臂的ROS功能包[universal_robot](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel)；
* 此外在`robot_sim/package`中有一些需要用到的但我在上面进行过一些修改的包，如解决gazebo中抓取物体会莫名抖动的包[gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs)，大寰机器人二指抓手AG-95的ROS功能包[dh_gripper_ros](https://github.com/DH-Robotics/dh_gripper_ros)以及其他依赖等。
* 非常感谢以上作者的无私奉献。
  
以下是如何安装和搭建本ROS功能包

```
cd ~/catkin_ws/src
git clone -b kinetic-devel https://github.com/pal-robotics/aruco_ros                #aruco_ros
git clone https://github.com/atenpas/gpd_ros/                                       #gpd_ros
git clone https://github.com/IFL-CAMP/easy_handeye                                  #easy_handeye
cd easy_handeye
git reset --hard 64b8b88                                                            #使用的是这个版本
cd ..
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git    #universal_robot
git clone https://github.com/Suyixiu/robot_sim                                      #本功能包
rosdep install --from-paths src --ignore-src --rosdistro=kinetic                    #安装依赖
catkin_make
```

## 在仿真环境中进行相机标定
首先是将相机模型还有标定板模型load进来
```
roslaunch robot_sim camera_calibration.launch
```
之后便可以看到RealSense D435i RGBD相机与标定板，其中这里使用的是7x6内角点，块块大小0.01m的标定板。若想使用其他尺寸的标定板可更改`experiment/camera_calibration/urdf/create_chessboard.py`脚本中的标定板参数后运行以在`experiment/camera_calibration/urdf`目录下生成你所需要的标定板，随后修改`camera_calibration.launch`要load的标定板即可。
<img src="https://z3.ax1x.com/2021/06/01/2uCb9A.jpg" width = "600" />  
<!-- <center><img src="https://z3.ax1x.com/2021/06/01/2uCb9A.jpg" width = "700" /></center>   -->

其中相机的URDF文件中使用的相机插件设置的相机视角是57°，图像分辨率是1280x720，所以根据相机内参各个参数的的定义算出该相机模型的内参真实值。这个计算与`camera_info`这个topic中的信息一致。
<p align="center">
<img src="https://latex.codecogs.com/gif.latex?\dpi{120}&space;\begin{aligned}&space;f_x=f_y&=\frac{image\_width&space;/&space;2}{tan(fov&space;/&space;2)}&space;=&space;\frac{1280&space;/&space;2}{tan(57&space;/&space;2)}&space;=&space;1178.73\\&space;c_x&space;&=&space;image\_width&space;/&space;2&space;=&space;1280&space;/&space;2&space;=&space;640\\&space;c_y&space;&=&space;image\_height&space;/&space;2&space;=&space;720&space;/&space;2&space;=360&space;\end{aligned}" title="\begin{aligned} f_x=f_y&=\frac{image\_width / 2}{tan(fov / 2)} = \frac{1280 / 2}{tan(57 / 2)} = 1178.73\\ c_x &= image\_width / 2 = 1280 / 2 = 640\\ c_y &= image\_height / 2 = 720 / 2 =360 \end{aligned}" />
</p>  

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

## 在仿真环境中进行深度图配准
此实验将用到实验一中采集的两个相机拍摄的标定板的图片，这里我们提供了一个python脚本`depth_image_registration.py`来计算配准矩阵并将其前两行存放在`Registration matrix.txt`中，因为实际remap深度图的时候也只会用到前两行。
```
python3 depth_image_registration.py
```
随后可以随便拿一对RGB图与深度图来观察配准矩阵对不对，因为后面会用到这个矩阵所以这里直接就是写成CPP了。
```
g++ ./depth_image_registration.cpp -o depth_image_registration $(pkg-config --cflags --libs opencv)
./depth_image_registration
```
下面是配准前与配准后的区别。  
<img src="https://z3.ax1x.com/2021/06/01/2Km3OH.png" width = "800" />  

## 在仿真环境中进行手眼标定
这里使用的手眼系统属于眼在手上的情况，即eye in hand，首先将我们提供的机械臂的moveit功能包跑起来，其中加载了UR10机械臂、大寰机器人的AG-95二指抓手
```
roslaunch yixiuge_ur10_moveit_config yixiuge_ur_moveit.launch
```
需要等moveit加载完之后再加载手眼标定包。
```
roslaunch robot_sim hand_eye_calibration.launch
```

## 在仿真环境中抓取
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

## 在仿真环境中进行数据采集
```
```

## 总结
