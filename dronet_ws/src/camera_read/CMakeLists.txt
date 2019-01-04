cmake_minimum_required(VERSION 2.8.3)
project(camera_read)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  image_transport 
  cv_bridge
)

find_package(OpenCV REQUIRED)

include_directories(
  include
  include/camera_read
  ${catkin_INCLUDE_DIRS}
 # "/usr/include/eigen3"
)

catkin_package()

# add the publisher example  

add_executable(test_image_subscriber src/test_image_subscriber.cpp)  
target_link_libraries(test_image_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})  

add_executable(test_image_publisher src/test_image_publisher.cpp)  
target_link_libraries(test_image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})  

