#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <my_image_transport/Box.h>

using namespace cv;
using namespace std;

bool drawing_box = false;
bool gotBB = false;
Rect box;

// bounding box mouse callback  
void mouseHandler(int event, int x, int y, int flags, void *param){
	switch (event){
	case CV_EVENT_MOUSEMOVE:
		if (drawing_box){
			box.width = x - box.x;
			box.height = y - box.y;
		}
		break;
	case CV_EVENT_LBUTTONDOWN:
		drawing_box = true;
		box = Rect(x, y, 0, 0);
		break;
	case CV_EVENT_LBUTTONUP:
		drawing_box = false;
		if (box.width < 0){
			box.x += box.width;
			box.width *= -1;
		}
		if (box.height < 0){
			box.y += box.height;
			box.height *= -1;
		}
		gotBB = true;
		break;
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat frame;
  try
  {
	if (!gotBB)
	{
		frame = cv_bridge::toCvShare(msg, "bgr8")->image;
		rectangle(frame, box, Scalar(0, 0, 255), 2);
		cv::imshow("view", frame);
		if (cvWaitKey(10) == 27)
			return;
	}
	//Remove callback  
	else
        {
	  //cvSetMouseCallback("Image", NULL, NULL);
          frame = cv_bridge::toCvShare(msg, "bgr8")->image;
          cv::imshow("view", frame);
	  if (cvWaitKey(10) == 27)
	  return;
        }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cvSetMouseCallback("view", mouseHandler, NULL);

  cv::startWindowThread();
  ros::Publisher box_publisher_;
  box_publisher_ = nh.advertise<my_image_transport::Box>("init_box", 1);
  my_image_transport::Box msg;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/left/image_rect_color", 1, imageCallback);
  
  int count=1;
  int rate = 25;//与发送端一致
  ros::Rate loop_rate(rate);
  while (nh.ok()) {
    if(gotBB && count <= 1)
    {
      cout << "Got the box:" << endl;
      cout << "x: " << box.x << endl;
      cout << "y: " << box.y << endl; 
      cout << "height: " << box.height << endl;
      cout << "width: " << box.width << endl;
      msg.x = box.x;
      msg.y = box.y;
      msg.height = box.height;
      msg.width = box.width;
      box_publisher_.publish(msg);
      cout << "Sent the box " << count++ << " times" << "! Start tracking object!" << endl;
      //sub.shutdown();
      //cout << "sub is shutdown!" << endl;
    }
      ros::spinOnce();
      loop_rate.sleep();
  }

  //ros::spin();
  cv::destroyWindow("view");
}
