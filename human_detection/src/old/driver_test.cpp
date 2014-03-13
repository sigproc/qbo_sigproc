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

#include "config.h"

/*********************************************************************************
**************************  Current Comments  ************************************






*********************************************************************************
*********************************************************************************/

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Full Segmentation";
static const char WNORMALIM[] = "Normal Image";
static const char WDEPTHSEG[] = "Depth Segmentation";
static const char WNORMALSEG[] = "Normal Segmentation";
static const char WDEPTHIM[] = "Depth Image";
static const char WCANDIDATES[] = "Candidates";
 
//prototype
int segment(float sigma, float k, int min_size, cv::Mat image);

/**********************************************8


***********************************************/

class Human_Detector
{
  /*ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;*/

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber sub_;
	//Use method of ImageTransport to create all publishers
	image_transport::Publisher pub_;
	image_transport::Publisher pubNormalIm_;
	image_transport::Publisher pubDepthSeg_;
	image_transport::Publisher pubNormalSeg_;
	image_transport::Publisher pubcandidates_;

  
public:
  Human_Detector()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    sub_ =it_.subscribe("/camera/depth/image_raw", 1, &Human_Detector::imageCallback, this);

	pub_ = it_.advertise("/human_detection/segmentation", 1);
	pubNormalIm_ = it_.advertise("/human_detection/normal", 1);
	pubDepthSeg_ = it_.advertise("/human_detection/depth_Seg",1);
	pubNormalSeg_ = it_.advertise("/human_detection/normal_seg",1);
	pubcandidates_ = it_.advertise("/human_detection/candidates", 1);

    //OpenCV HighGUI call to create a display window on start-up.
#if SHOWJOINTSEG
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
#endif

#if SHOWNORMALIM
    cv::namedWindow(WNORMALIM, CV_WINDOW_AUTOSIZE);
#endif

#if SHOWDEPTHSEG
	cv::namedWindow(WDEPTHSEG, CV_WINDOW_AUTOSIZE);
#endif

#if SHOWNORMALSEG
	cv::namedWindow(WNORMALSEG, CV_WINDOW_AUTOSIZE);
#endif

#if SHOWCANDIDATES
	cv::namedWindow(WCANDIDATES, CV_WINDOW_AUTOSIZE);
#endif
    
  }

  ~Human_Detector()
  {
	#if SHOWJOINTSEG
		cv::destroyWindow(WINDOW);
	#endif

	#if SHOWNORMALIM
		cv::destroyWindow(WNORMALIM);
	#endif

	#if SHOWDEPTHSEG
		cv::destroyWindow(WDEPTHSEG);
	#endif

	#if SHOWNORMALSEG
		cv::destroyWindow(WNORMALSEG);
	#endif

	#if SHOWCANDIDATES
		cv::destroyWindow(WCANDIDATES);
	#endif

	//cv::destroyAllWindows();
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
 
	const char* filename = "tmpIm.ppm";
	float sigma = SIGMA;
	float Kdepth = KDEPTH;
	float Knormal = KNORMAL;
	int min_size = MIN_SIZE;
	int alpha = ALPHA, s = ESS;

	//params for conversion
	int in_width, in_height;
	int sub_width, sub_height;
	in_width = in_msg->image.size().width;
	in_height = in_msg->image.size().height;
	int draw = 0;
	float dval = 0;
	//create seed for random numb generator
	srand(time(NULL));

	//subsample variables
	sub_width = in_width/alpha;
	sub_height = in_height/alpha;

	//direct conversion from Mat to image<rgb>
	image<int> *inputIm = new image<int>(sub_width,sub_height);

	int maxraw;


	//THIS IS A MESS!
	for (int y=0; y < sub_height; y++){
		for(int x=0; x < sub_width; x++){
			draw = in_msg->image.at<unsigned short>(y,x);
			std::cout << "RawRaw (" << x << "," << y << "): " << draw << std::endl;
			//grab first 11 bits
			draw = draw & 0x0FFF;
			std::cout << "Raw (" << x << "," << y << "): " << draw << std::endl;
			dval = (8*0.075*F_HPELS)/(1084 - draw) ;
			std::cout << "Metric (" << x << "," << y << "): " << dval << std::endl;	
		}
	}

	
	
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

