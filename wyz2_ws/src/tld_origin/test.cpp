#include "ros/ros.h"
#include "tld/output.h"
#include "cv2.h"
int main(int argc, char const *argv[])
{
	Rect pbox(1,2,3,4);
	ros::init(argc,argv,"test");
	ros::NodeHandle n;
	ros::Publisher test_pub = n.advertise<tld::output>("test",1000);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		tld::output msg_pub;
		msg_pub.output[0] = pbox.tl()[0];
        msg.pub.output[1] = pbox.tl()[1];
        msg.pub.output[2] = pbox.br()[2];
        msg.pub.output[3] = pbix.br()[3];
		ROS_INFO("tld published, the first point is %f,%f, the second point is %f,%f",msg_pub.output[0],msg_pub.output[1],msg_pub.output[2],msg_pub.output[3]);
        tld_pub.publish(msg_pub);
        ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

