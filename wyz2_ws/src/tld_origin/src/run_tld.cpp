#include <opencv2/opencv.hpp>
#include <tld_utils.h>
#include <iostream>
#include <sstream>
#include <TLD.h>
#include <stdio.h>
//ros
#include <ros/ros.h>  
#include "tld_origin/output.h"
using namespace cv;
using namespace std;
//Global variables
Rect box;
bool drawing_box = false;
bool gotBB = false;
bool tl = true;
bool rep = false;
bool fromfile=false;
string video;
//ros Global variables
bool ros_enable = true;
bool ros_recieved = false;


void readBB(char* file){
  ifstream bb_file (file);
  string line;
  getline(bb_file,line);
  istringstream linestream(line);
  string x1,y1,x2,y2;
  getline (linestream,x1, ',');
  getline (linestream,y1, ',');
  getline (linestream,x2, ',');
  getline (linestream,y2, ',');
  int x = atoi(x1.c_str());// = (int)file["bb_x"];
  int y = atoi(y1.c_str());// = (int)file["bb_y"];
  int w = atoi(x2.c_str())-x;// = (int)file["bb_w"];
  int h = atoi(y2.c_str())-y;// = (int)file["bb_h"];
  box = Rect(x,y,w,h);
}

//save images
string outFlie = "/home/zq610/WYZ/media/image_got/";
bool SAVE = false;

//bounding box mouse callback
void mouseHandler(int event, int x, int y, int flags, void *param){
  switch( event ){
  case CV_EVENT_MOUSEMOVE:
    if (drawing_box){
        box.width = x-box.x;
        box.height = y-box.y;
    }
    break;
  case CV_EVENT_LBUTTONDOWN:
    drawing_box = true;
    box = Rect( x, y, 0, 0 );
    break;
  case CV_EVENT_LBUTTONUP:
    drawing_box = false;
    if( box.width < 0 ){
        box.x += box.width;
        box.width *= -1;
    }
    if( box.height < 0 ){
        box.y += box.height;
        box.height *= -1;
    }
    gotBB = true;
    break;
  }
}

void print_help(char** argv){
  printf("use:\n     %s -p /path/parameters.yml\n",argv[0]);
  printf("-s    source video\n-b        bounding box file\n-tl  track and learn\n-r     repeat\n");
}

void read_options(int argc, char** argv,VideoCapture& capture,FileStorage &fs){
  for (int i=0;i<argc;i++){
      if (strcmp(argv[i],"-b")==0){
          if (argc>i){
              readBB(argv[i+1]);
              gotBB = true;
          }
          else
            print_help(argv);
      }
      if (strcmp(argv[i],"-s")==0){
          if (argc>i){
              video = string(argv[i+1]);
              capture.open(video);
              fromfile = true;
          }
          else
            print_help(argv);

      }
      if (strcmp(argv[i],"-p")==0){
          if (argc>i){
              fs.open(argv[i+1], FileStorage::READ);
          }
          else
            print_help(argv);
      }
      if (strcmp(argv[i],"-no_tl")==0){
          tl = false;
      }
      if (strcmp(argv[i],"-r")==0){
          rep = true;
      }
	  if ( strcmp (argv[i], "-save") == 0){// save images
		  SAVE = true;
	  }
  }
}

//ros call back
void fasterrcnnCallback(const tld_origin::output msg){
	ROS_INFO("I heared :[%f] [%f] [%f] [%f]",msg.output[0], msg.output[1], msg.output[2], msg.output[3]);
	box = Rect((int)msg.output[0], (int)msg.output[1], (int)(msg.output[2]-msg.output[0]), (int)(msg.output[3]-msg.output[1]));
	ros_recieved = true;
}

int main(int argc, char * argv[]){
  //ros publisher initial
  // if(ros_enable){
  	ros::init(argc, argv, "tld_tracking");
  	ros::NodeHandle n;
  	ros::Publisher tld_pub = n.advertise<tld_origin::output>("tld_tracking", 1000);
  	ros::Rate loop_rate(10);
	  tld_origin::output msg_pub;
  // }

  VideoCapture capture;
  capture.open(0);
  FileStorage fs;
  //Read options
  read_options(argc,argv,capture,fs);
  //Init camera
  if (!capture.isOpened())
  {
	cout << "capture device failed to open!" << endl;
    return 1;
  }
  //Register mouse callback to draw the bounding box
  cvNamedWindow("TLD",CV_WINDOW_AUTOSIZE);//----------------
  //using ros not hand selecting
  //cvSetMouseCallback( "TLD", mouseHandler, NULL );//----------
  //TLD framework
  TLD tld;
  //Read parameters file
  tld.read(fs.getFirstTopLevelNode());
  Mat frame;
  Mat last_gray;
  Mat first;
  if (fromfile){
      capture >> frame;
      cvtColor(frame, last_gray, CV_RGB2GRAY);
      frame.copyTo(first);
  }else{
      capture.set(CV_CAP_PROP_FRAME_WIDTH,600);	//resolution
      capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  }

  ///Initialization
GETBOUNDINGBOX:
//get initial bbox from fasterrccn by ros
	if(ros_enable)
  {
    ros::init(argc,argv,"tld");  
    ros::NodeHandle nh;        
    ros::Subscriber sub = nh.subscribe("fasterrcnn",10, fasterrcnnCallback);
    ros::Rate loop_rate(5);
    while(!ros_recieved)
    {
		    ros::spinOnce();
      	loop_rate.sleep();
    }
	//capture >> frame;
	//cvtColor(frame, last_gray, CV_RGB2GRAY);  
	//drawBox(frame,box);
	//imshow("TLD", frame); 
  }
  if(!gotBB)	//not detection
  {
		if (!fromfile){
		  capture >> frame;
		}
		else
		  first.copyTo(frame);
		cvtColor(frame, last_gray, CV_RGB2GRAY);
		drawBox(frame,box);
		imshow("TLD", frame);
		if (cvWaitKey(33) == 'q')
			return 0;
	}
  if (min(box.width,box.height)<(int)fs.getFirstTopLevelNode()["min_win"]){
	  cout << "Bounding box too small, try again." << endl;
	  gotBB = false;
	  goto GETBOUNDINGBOX;
  }
	
  //Remove callback
  //using ros not hand selecting
  //cvSetMouseCallback( "TLD", NULL, NULL );
  printf("Initial Bounding Box = x:%d y:%d h:%d w:%d\n",box.x,box.y,box.width,box.height);  //warning!!not a.x a.y b.x b.y. it is x y w d
  //Output file
  FILE  *bb_file = fopen("bounding_boxes.txt","w");
  //TLD initialization
  tld.init(last_gray,box,bb_file);

  ///Run-time
  Mat current_gray;
  BoundingBox pbox;
  vector<Point2f> pts1;
  vector<Point2f> pts2;
  bool status=true;
  int frames = 1;
  int detections = 1;
REPEAT:
  while(capture.read(frame)){
    //get frame
    cvtColor(frame, current_gray, CV_RGB2GRAY);
    //Process Frame
    tld.processFrame(last_gray,current_gray,pts1,pts2,pbox,status,tl,bb_file);
    //Draw Points
    if (status){
      drawPoints(frame,pts1);
      drawPoints(frame,pts2,Scalar(0,255,0));
      drawBox(frame,pbox);
      detections++;
    }
    //Display
    imshow("TLD", frame);
	if (ros_enable){
    printf("enter the first if\n");//////////////////////////////////////////
		msg_pub.output[0] = pbox.tl().x;
		msg_pub.output[1] = pbox.tl().y;
		msg_pub.output[2] = pbox.br().x;	
		msg_pub.output[3] = pbox.br().y;
		ROS_INFO("tld published, the first point is %f,%f, the second point is %f,%f",msg_pub.output[0],msg_pub.output[1],msg_pub.output[2],msg_pub.output[3]); 	
		tld_pub.publish(msg_pub);
		ros::spinOnce();
	}

	//save images
	stringstream buf;
	buf << frames;
	string num = buf.str();
	if(SAVE){
		imwrite(outFlie + num + ".jpg", frame);
	}

    //swap points and images
    swap(last_gray,current_gray);
    pts1.clear();
    pts2.clear();
    frames++;
    printf("Detection rate: %d/%d\n",detections,frames);
    if (cvWaitKey(33) == 'q')
      break;
  }
  if (rep){
    rep = false;
    tl = false;
    fclose(bb_file);
    bb_file = fopen("final_detector.txt","w");
    //capture.set(CV_CAP_PROP_POS_AVI_RATIO,0);
    capture.release();
    capture.open(video);
    goto REPEAT;
  }
  fclose(bb_file);
  return 0;
}
