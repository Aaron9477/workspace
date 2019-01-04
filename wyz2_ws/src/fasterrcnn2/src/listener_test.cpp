    #include <ros/ros.h>  
    #include "std_msgs/String.h"  
	#include "fasterrcnn2/output.h"
      
    void chatcallback(const fasterrcnn2::output msg)//回调函数  
    {  
        ROS_INFO("I heared :[%f] [%f] [%f] [%f]",msg.output[0], msg.output[1], msg.output[2], msg.output[3]);  
    }  
    int main(int argc,char **argv)  
    {  
        //ros::init(argc,argv,"listener"); 
		ros::init(argc,argv,"test");  
        ros::NodeHandle nh;  
      
        ros::Subscriber sub = nh.subscribe("fasterrcnn",1000,chatcallback);  
        ros::spin();  
        return 0;  
    }  
