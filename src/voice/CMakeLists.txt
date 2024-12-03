cmake_minimum_required(VERSION 3.0.2)
project(voice)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Declare a C++ executable
add_executable(iat_online_record_sample src/iat_online_record_sample.cpp)

## 添加库文件路径
link_directories(/home/ros/chenxu/educational_robot/libs)

## 添加头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/ros/chenxu/educational_robot/include  # 如果头文件位于这个目录
)

## 链接 msc 库文件
target_link_libraries(iat_online_record_sample
  ${catkin_LIBRARIES}
  msc  # 确保连接到 libmsc.so
  pthread  # 如果 libmsc.so 需要 pthread 库
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES voice
  # CATKIN_DEPENDS roscpp rospy std_msgs
  # DEPENDS system_lib
)

###########
## Build ##
###########
