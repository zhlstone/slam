cmake_minimum_required(VERSION 2.6)
project(point_surface)

find_package(Eigen3  REQUIRED)
include_directories( ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(PCL REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(point_surface main.cpp)
target_link_libraries(point_surface ${PCL_LIBRARIES})

install(TARGETS point_surface RUNTIME DESTINATION bin)
