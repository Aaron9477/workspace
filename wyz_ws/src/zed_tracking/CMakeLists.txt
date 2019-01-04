cmake_minimum_required(VERSION 2.8.3)
project(zed_tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)


##For Jetson, OpenCV4Tegra is based on OpenCV2.4

#elseif(CMAKE_SYSTEM_NAME2 MATCHES "armv7l" ) # K1

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Box.msg
   zed_roi.msg
 )
## Generate added messages and services with any dependencies listed here
 generate_messages(
 )
#generate_dynamic_reconfigure_options(
#  cfg/Zed.cfg
#)
###################################
## catkin specific configuration ##
###################################
catkin_package(

)

###############################################################################
# INCLUDES

# Specify locations of header files.


###############################################################################

###############################################################################
# EXECUTABLE








