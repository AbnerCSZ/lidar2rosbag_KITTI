cmake_minimum_required(VERSION 2.8.3)
project(lidar2rosbag)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  rosbag_storage
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs rosbag_storage
  DEPENDS EIGEN3 PCL OpenCV
  
 
)

add_executable(lidar2rosbag src/lidar2rosbag.cpp)
target_link_libraries(lidar2rosbag ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})




