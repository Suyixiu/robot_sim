# 基于ROS搭建一个仿真环境
写写文档。

| Author | 苏一休            |
| ------ | ----------------- |
| E-mail | 2270678755@qq.com |
![image](https://ae01.alicdn.com/kf/H676fffff7c384439aaa4124f082647994.gif)

## 实验一：camera_calibration
首先是将相机还有标定板load进来
```
roslaunch robot_sim camera_calibration.launch
```
可以使用ROS自带的标定包来进行标定，
```
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.01 image:=/camera/rgb/image_raw camera:=/camera/rgb     #RGB相机
rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.01 image:=/camera/ir/image_raw camera:=/camera/ir       #IR相机
```
接下来是移动标定板，程序中设定的是1秒钟产生一个随机位置并移动标定板，若相机能正确识别出标定板角点则将此时的照片保存，有深度图，红外相机的图还有RGB图存放在`save_checkboard_img`目录中
```
rosrun robot_sim camera_calibration
```
在获取到RGB相机与IR相机的图像之后可使用matlab进行内参的计算

还可用张振友标点发进行内参的计算，这里我们提供了一个python的脚本`camera_calibration.py`用于载入前面保存的图片以计算RGB相机与IR相机的内参并分别保存在`IR_cameraintrinsic_parameters.npz`与`RGB_cameraintrinsic_parameters.npz`中，直接python运行此脚本即可
```
python3 camera_calibration.py
```

## 实验二：depth_image_registration
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