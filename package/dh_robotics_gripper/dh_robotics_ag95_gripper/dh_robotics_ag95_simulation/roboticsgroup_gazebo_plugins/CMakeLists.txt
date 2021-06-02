cmake_minimum_required(VERSION 2.8.3)
project(roboticsgroup_gazebo_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  control_toolbox
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros
    control_toolbox
)

add_definitions(-std=gnu++0x)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

add_library(roboticsgroup_gazebo_mimic_joint_plugin src/mimic_joint_plugin.cpp)
target_link_libraries(roboticsgroup_gazebo_mimic_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(roboticsgroup_gazebo_disable_link_plugin src/disable_link_plugin.cpp)
target_link_libraries(roboticsgroup_gazebo_disable_link_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

