cmake_minimum_required(VERSION 2.8.3)

if(COMMAND cmake_policy)
     cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(tld)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(OpenCV 3.2)


add_message_files(
	FILES
	output.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set the include directories

catkin_package(
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    opencv
    image_transport
    message_runtime
)

include_directories (${PROJECT_SOURCE_DIR}/include	${OpenCV_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

add_library(tld_utils ${PROJECT_SOURCE_DIR}/src/tld_utils.cpp)
add_library(LKTracker ${PROJECT_SOURCE_DIR}/src/LKTracker.cpp)
add_library(ferNN ${PROJECT_SOURCE_DIR}/src/FerNNClassifier.cpp)
add_library(tld ${PROJECT_SOURCE_DIR}/src/TLD.cpp)

add_executable(run_tld ${PROJECT_SOURCE_DIR}/src/run_tld.cpp)
add_executable(run_tld_ros ${PROJECT_SOURCE_DIR}/src/run_tld_ros.cpp)
#link the libraries
target_link_libraries(run_tld tld LKTracker ferNN tld_utils  ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(run_tld_ros tld LKTracker ferNN tld_utils  ${catkin_LIBRARIES} ${OpenCV_LIBS})
#set optimization level 
add_dependencies(run_tld tld_generate_messages_cpp)
add_dependencies(run_tld_ros tld_generate_messages_cpp)
set(CMAKE_BUILD_TYPE Release)



