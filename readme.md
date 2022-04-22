# A ROS based Open Source Simulation Environment for Robotics Beginners
基于ROS搭建的机器人仿真环境。其中设计了机器人研究过程中需要预先进行的几个重要实验如各种标定(相机标定，深度图配准，手眼标定)，还在仿真环境中实现了两种抓取算法的仿真，一种是传统方法(几何方法)一种是机器学习的方法(GPD)，最后一个实验是模拟机械臂采集数据的场景。  
视频链接-->[Bilibili](https://www.bilibili.com/video/BV19f4y1h73E)&[YouTube](https://youtu.be/Ky5vFTKUd1w)  

| Author | 苏一休            |
| ------ | ----------------- |
| E-mail | 2270678755@qq.com |

## 目录
  - [搭建](#搭建)
  - [在仿真环境中进行相机标定](#在仿真环境中进行相机标定)
  - [在仿真环境中进行深度图配准](#在仿真环境中进行深度图配准)
  - [在仿真环境中进行手眼标定](#在仿真环境中进行手眼标定)
  - [在仿真环境中抓取](#在仿真环境中抓取)
    - [基于几何方法的抓取](#基于几何方法的抓取)
    - [基于机器学习方法的抓取](#基于机器学习方法的抓取)
  - [在仿真环境中进行数据采集](#在仿真环境中进行数据采集)

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
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=kinetic                    #安装依赖
catkin_make

cp -r ./src/robot_sim/experiment/hand_eye_calibration/urdf/aruco/ ~/.gazebo/models  #复制aruco模型到gazebo默认模型文件夹中
```

## 在仿真环境中进行相机标定
首先是将相机模型还有标定板模型load进来
```
cd ~/your_catkin_ws/
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
cd ~/your_catkin_ws/
rosrun robot_sim camera_calibration
```
这里我们提供了一个python的脚本`camera_calibration.py`，位于`robot_sim/experiment/camera_calibration/scripts`，用于载入前面保存的图片以计算RGB相机与IR相机的内参并分别保存在`IR_cameraintrinsic_parameters.npz`与`RGB_cameraintrinsic_parameters.npz`中，直接python运行此脚本即可。你也可以自己写程序来计算内参，并与前面公式计算的结果像对比以验证你的标定算法的准确性与误差。
```
cd ~/your_catkin_ws/src/robot_sim/experiment/camera_calibration/scripts
python3 camera_calibration.py
```

## 在仿真环境中进行深度图配准
此实验将用到实验一中采集的两个相机拍摄的标定板的图片，这里我们提供了一个python脚本`depth_image_registration.py`来计算配准矩阵并将其前两行存放在`Registration_matrix.txt`中，因为实际remap深度图的时候也只会用到前两行。
```
cd ~/your_catkin_ws/src/robot_sim/experiment/depth_image_registration/scripts
python3 depth_image_registration.py
```
随后可以随便拿一对RGB图与深度图来观察配准矩阵对不对，因为后面会用到这个矩阵所以这里直接就是写成CPP了。
```
cd ../src
g++ ./depth_image_registration.cpp -o depth_image_registration $(pkg-config --cflags --libs opencv)
./depth_image_registration
```
下面是配准前与配准后的区别。  
<img src="https://z3.ax1x.com/2021/06/01/2Km3OH.png" width = "800" />  

## 在仿真环境中进行手眼标定
这里使用的手眼系统属于眼在手上的情况，即eye on hand，首先将我们提供的机械臂的moveit功能包跑起来，其中加载了UR10机械臂、大寰机器人的AG-95二指抓手还有D435i RGBD相机安装在UR10机械臂的末端。这里的手的link是`yixiuge_ee_link`，眼的link是相机的RGB光学frame`camera_rgb_optical_frame`。标定用到的Aruco二维码的大小是0.2m，ID是582。
```
roslaunch yixiuge_ur10_moveit_config yixiuge_ur_moveit.launch
```
需要等moveit加载完之后再加载手眼标定包，因为大寰的二指抓手开起来有点费时，如果马上开hand_eye_calibration的话可能因为找不到moveit的group而报错。
```
roslaunch robot_sim hand_eye_calibration.launch
```
<img src="https://z3.ax1x.com/2021/06/02/2lpBb6.png" width = "800" />  

之后请参照视频中的操作进行操作最后可以算出手眼矩阵。设计的17个点一般情况下不会全部满足，可以通过RVIZ中的moveit的球拖动机械臂渠道其他位置以采集更多的数据从而使标定结果更加准确，下面是我们采集了37个点之后所计算出来的结果
```
translation: 
  x: -0.0171515439439
  y: 0.129039200607
  z: 0.146263405556
rotation: 
  x: 0.999995982804
  y: 0.00268604823595
  z: 0.000687382040816
  w: 0.000589089946183
```
提供的机器人URDF文件中手眼矩阵的真实值如下，可以看出还是很准确的，误差在3,4毫米这样子。
```
translation: 
  x: -0.0175
  y: 0.128
  z: 0.1425
rotation: 
  x: 1
  y: 0
  z: 0
  w: 0
```
手眼标定完成点击save后数据会保存在`~/.ros/easy_handeye/`的`easy_handeye_eye_on_hand.yaml`中，运行下面的launch文件来观察标定的结果
```
roslaunch yixiuge_ur10_moveit_config yixiuge_ur_moveit.launch
roslaunch robot_sim hand_eye_calibration_result.launch
```
<img src="https://z3.ax1x.com/2021/06/02/2lKUR1.png" width = "500" />    

## 在仿真环境中抓取
这里设计了两种方法的抓取，一种是使用传统的几何方法识别抓取点的抓取([参考论文](https://link.springer.com/article/10.1023/A:1008381314159))，一种是基于机器学习的抓取([GPD](https://github.com/atenpas/gpd))
### 基于几何方法的抓取  
先把机械臂和抓取环境load进来随后启动抓取。
```
roslaunch robot_sim grasp_world.launch 
cd ~/your_catkin_ws/                                #这是必要的
rosrun robot_sim geometric_method_grasp             #因为这里要加载mask所以路径要对
```
我们这里提供的方法主要原理是先将物体从桌面分割，这里的分割使用的是直接将相机当前画面减去一个mask，然后设置阈值二值化。因为是仿真环境所以这样做还是比较稳定的，分割效果也是没有问题的。但这种超级简单的做法仅限于环境固定的情况，实际使用的话还是建议使用其他分割算法，比如用深度图生成掩模来提取物体。  
<img src="https://z3.ax1x.com/2021/06/03/283gmR.png" width = "900" />    

分割出物体之后便是滤波然后提取轮廓，膨胀后再腐蚀以连接成闭合的轮廓，随后选择最大的轮廓作为机械臂要抓取的物体。计算轮廓的一阶矩得到物体的重心，这里假设物体的质量分布均匀。随后使用PCA计算轮廓的主方向和垂直主方向的副方向，这里也可使用轮廓的二阶矩来获得。随后计算轮廓的最小外接矩形，与副轴的两个交点连成一条直线，随后遍历这条直线上的点找到副轴与轮廓的两个交点作为抓取点，随后绘制抓取矩形框。详细实现请见代码。
<img src="https://z3.ax1x.com/2021/06/03/288IU0.png" width = "900" />  


### 基于机器学习方法的抓取
这里使用的是atenpas的[GPD](https://github.com/atenpas/gpd)这个方法，具体原理可见[论文](https://arxiv.org/abs/1706.09911)，这里不再赘述。在搭建GPD库的时候你可能会遇到某些问题，倘若GPD2.0.0版本出现问题则建议使用GPD1.5.0。这里在我们的仿真环境中成功运行了这一算法。我们提供的代码使用的是GPD的sample方式，使用的是caffe的cfg文件。再运行这部分代码的时候请注意调整cfg文件到正确的路径。此外这里的分割使用的是根据距离风格，而实际更为鲁棒的做法应该是使用RGB图生成掩模来分割点云。
```
roslaunch robot_sim grasp_world.launch
roslaunch robot_sim gpd_run.launch type:=2 topic:=/cloud_sample
rosrun robot_sim GPD_method_grasp
```
<img src="https://z3.ax1x.com/2021/06/03/28YngS.png" width = "900" />  

### 验证你的GPD是否安装正确  
GPD的安装跟使用很蛋疼，如果上面的GPD抓取有报错很有可能是你GPD没有装对。下面提供了一个测试你的GPD是否可用的包。
```
roslaunch robot_sim gpd_run.launch type:=2 topic:=/cloud_sample
roslaunch robot_sim test_gpd.launch
```
如果你不想使用GPD那就直接把CMakeLists里面的有关GPD的都注释掉即可。

### 关于gazebo中抓手抓取物体会莫名抖动
这是因为抓手的控制是位置控制而不是力控制所造成的。我们是使用JenniferBuehler编写的gazebo插件[gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs)来解决这个问题的。具体原理是插件会检测手指与物体接触，设定一些阈值当达到条件之后将物体与抓手的相对位置进行固定并失能物体的collision属性从而解决这一抖动的问题。

## 在仿真环境中进行数据采集
主要原理是围绕物体生成若干个位姿，随后驱动机械臂运动到指定位姿之后拍照，保存数据
```
roslaunch robot_sim data_collection.launch
cd ~/your_catkin_ws/                                #这是必要的
rosrun robot_sim data_collection
```
其中采样点的多少以及位置等都是可以认为调节的，以下是56个的采样点
<img src="https://z3.ax1x.com/2021/06/03/28Y7PP.png" width = "900" />  

