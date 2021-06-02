cmake_minimum_required(VERSION 2.8.3)
project(robot_sim)

set( CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  cv_bridge
  image_transport
  roscpp
  serial
  std_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  control_msgs 
  move_base_msgs
  trajectory_msgs
  cmake_modules
  moveit_core
  moveit_msgs
  moveit_visual_tools
  moveit_ros_planning_interface
  tf
  geometry_msgs
  pcl_conversions
  pcl_ros
)

generate_dynamic_reconfigure_options(
  ${PROJECT_SOURCE_DIR}/grasp/config/grasp.cfg
  ${PROJECT_SOURCE_DIR}/grasp/config/gpd.cfg
  ${PROJECT_SOURCE_DIR}/data_collection/config/data_collection.cfg
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  dynamic_reconfigure
    roscpp
    serial
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs actionlib
    move_base_msgs
    control_msgs
    trajectory_msgs
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_perception
    moveit_ros_planning_interface
    pluginlib
    actionlib_msgs
)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package( OpenCV 3.3.1 REQUIRED )

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "/usr/include/eigen3"
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS} 
)

# 相机标定部分  #
aux_source_directory(${PROJECT_SOURCE_DIR}/camera_calibration/src camera_calibration_SrcFiles)
add_executable(camera_calibration ${camera_calibration_SrcFiles})
target_link_libraries(camera_calibration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# 使机器人回到初始位置  #
aux_source_directory(${PROJECT_SOURCE_DIR}/hand_eye_calibration/src reset_robot_pose_SrcFiles)
add_executable(reset_robot_pose ${reset_robot_pose_SrcFiles})
target_link_libraries(reset_robot_pose ${catkin_LIBRARIES} )

# 抓取部分 #
## geometric_method_grasp ##
include_directories(${PROJECT_SOURCE_DIR}/grasp/inc)
aux_source_directory(${PROJECT_SOURCE_DIR}/grasp/geometric_method/src geometric_method_SrcFiles)
add_executable(geometric_method_grasp ${geometric_method_SrcFiles})
add_dependencies(geometric_method_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geometric_method_grasp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## GPD_method_grasp ##
aux_source_directory(${PROJECT_SOURCE_DIR}/grasp/GPD_method/src GPD_method_SrcFiles)
add_executable(GPD_method_grasp ${GPD_method_SrcFiles})
add_dependencies(GPD_method_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(GPD_method_grasp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# 数据采集部分 #
aux_source_directory(${PROJECT_SOURCE_DIR}/data_collection/src data_collection_SrcFiles)
add_executable(data_collection ${data_collection_SrcFiles})
add_dependencies(data_collection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_collection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

