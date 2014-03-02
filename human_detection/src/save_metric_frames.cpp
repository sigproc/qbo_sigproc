#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>       /* isnan, sqrt */
#include <vector>
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include "segment_depth/segment.h"
#include "merge_and_filter/merge_and_filter.h"
#include "classification/descriptor.h"
#include "utilities.h"

#include "config.h"

std::string directory;
int frame_count;

/*********************************************************************************
**************************  Current Comments  ************************************




*********************************************************************************
*********************************************************************************/

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;




 
/**********************************************8


***********************************************/

class Human_Detector
{
 
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber sub_;

  
public:
  Human_Detector()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    sub_ =it_.subscribe("/camera/depth/image", 1, &Human_Detector::imageCallback, this);
	directory = "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/bag_frames/";

	//initialise frame count
	frame_count = 0;
	std::string dir = "trash";
	//try and grab "dir" value
	ros::NodeHandle p("~");
	p.getParam("dir", dir);
	//ros::param::get("dir", dir);
	std::cout << "dir is read as " << dir << std::endl;
	directory = directory+dir+"/";
	std::cout<< "Directory = $" << directory << std::endl;
	ROS_INFO("Saving Files in %s", directory.c_str());
	

  }

  ~Human_Detector()
  {
	
  }


	void imageCallback(const sensor_msgs::ImageConstPtr& original_image);
	

};

/***************************************************************

*********************************************************************/
//This function is called everytime a new image is published
void Human_Detector::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

    cv_bridge::CvImagePtr in_msg;

    try
    {
        in_msg = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("ROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 
	//image is stored as 32FC1 in in_msg->image which is of type Mat
	char f[50];

	std::string four_chars;
	if ( (frame_count/10000) == 0)		four_chars.append("0");
	if ( (frame_count/1000) == 0)		four_chars.append("0");
	if( (frame_count/100) == 0)			four_chars.append("0");
	if( (frame_count/10) == 0) 			four_chars.append("0");
	char buffer[10];
	sprintf(buffer, "%d", frame_count);
	four_chars.append(buffer);
	sprintf(f, "frame_%s.jsc68", four_chars.c_str());
	std::string filename = f;
	filename = directory+filename;

	writeMatToFile(in_msg->image, filename);
	frame_count++;
	
	/*
	cv::Mat inframe;
	readFileToMat(inframe, filename);

	int in_width = in_msg->image.size().width;
	int in_height = in_msg->image.size().height;

	float cumsum = 0;

	for (int y=0; y < in_height; y++){
		for(int x=0; x < in_width; x++){
			float loaded = inframe.at<float>(y,x);
			float msg = in_msg->image.at<float>(y,x);

			//std::cout << "At (" << x << "," << y << "): " << std::endl;
			//std::cout << "Msg image has val: " << msg;
			//std::cout << " and Loaded image has val: " << loaded << std::endl;
			if( !isnan(loaded) && !isnan(msg)){
				//std::cout << "Difference: " << msg-loaded << std::endl;
				cumsum+= msg-loaded;
			}
		}
	}

	std::cout << "difference between loaded image and message is " << cumsum <<  std::endl;
	std::cout << "Displaying image" << std::endl;

	cv::imshow("WINDOW", inframe);
	cv::waitKey(1000);*/
	
}




/********************************************************

*******************************************************/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Human_Detector");
  Human_Detector hd;
  ros::spin();
  return 0;
}

