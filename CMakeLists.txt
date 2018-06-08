cmake_minimum_required(VERSION 2.8.3)
project(easy_mcl)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED
COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_highgui
  CONFIG
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES easy_mcl
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  DEPENDS eigen opencv3
)

add_definitions(${EIGEN_DEFINITIONS})

add_executable(easy_mcl src/main.cpp)

target_link_libraries(easy_mcl
  ${catkin_LIBRARIES} 
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
