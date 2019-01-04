#include <ros/ros.h>  
#include "std_msgs/String.h"  
#include <iostream>  
#include <sstream>
#include "fasterrcnn2/output.h"
  
int main(int argc,char **argv)  
{  
    ros::init(argc,argv,"talker_test");   //ros系统初始化，通过命令对名字进行重映射，节点名字在一个运行的系统中必须是唯一的  
    ros::NodeHandle nh;//创建一个句柄，并自动初始化  
  
    ros::Publisher pub = nh.advertise<fasterrcnn2::output>("fasterrcnn",1000); //将std_msgs::String类型的消息发布给话题chatter，参数1000是消息队列的长度  
  
    ros::Rate rate(10);//指定自循环的频率  1/10  
    int count = 0;  
  
    while(ros::ok())  //当按下Ctrl+c命令时，返回false  
    {  
		fasterrcnn2::output bbox;
		bbox.output[0] = 120;		
		bbox.output[1] = 80;
		bbox.output[2] = 240;
		bbox.output[3] = 160;
        //std_msgs::String;  
        //std::stringstream ss;  
        //ss<< "Hello ROS!"<<count;  
        //msg.data =ss.str();  
  
        ROS_INFO("%f",bbox.output[0]);  
        pub.publish(bbox);  
        ros::spinOnce();  
        rate.sleep();  
        ++count;  
    }  
    return 0;  
}  
