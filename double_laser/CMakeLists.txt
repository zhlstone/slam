cmake_minimum_required(VERSION 2.8.3)
project(double_laser)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  message_filters
  rosconsole
   tf
  sensor_msgs
  cmake_modules
  laser_geometry
  tf_conversions
)

find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES double_laser
#  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(double_laser src/double_laser.cpp)
target_link_libraries(double_laser  ${catkin_LIBRARIES} ${PCL_LIBRARARIES}) 
