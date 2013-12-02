#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include "segment_depth/segment.h"

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//prototype
int segment(float sigma, float k, int min_size, cv::Mat image);

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
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
 
	//get the image into a known format and display
    double minval, maxval;
    cv::minMaxIdx(in_msg->image, &minval, &maxval);
    std::cout << "Minval: " << minval << std::endl;
    std::cout << "Maxval: " << maxval << std::endl;

    //cv::Mat *output_Im = new cv::Mat();
    in_msg->image.convertTo(in_msg->image, CV_8UC1,255.0/maxval);

	const char* filename = "tmpIm.ppm";
	float sigma = 0.5;
	float k = 500;
	int min_size = 20;

	//params for conversion
	int width, height;
	width = in_msg->image.size().width;
	height = in_msg->image.size().height;
	uchar dval = 0;

	//direct conversion from Mat to image<rgb>
	image<float> *inputIm = new image<float>(width,height);
	image<rgb> *outputIm = new image<rgb>(width,height);

	//Now scan through and create image<rbg> for segmenting
	for (int y=0; y < height; y++){
		for(int x=0; x<width; x++){
			//get values from
			uchar intensity = in_msg->image.at<uchar>(y,x);
			dval = intensity;
			//gval = intensity;
			//bval = intensity;
			imRef(inputIm,x,y) = dval;
			//imRef(inputIm,x,y).g = gval;
			//imRef(inputIm,x,y).b = bval;
		}
	}


	int num_ccs; 
	//segment image
	outputIm = segment_image1C(inputIm, sigma, k, min_size, &num_ccs);

	//create RGB Mat
	cv::Mat OutputBGRMat = cv::Mat::zeros(in_msg->image.size(),CV_8UC3);

	//fill in NEW mat with segmented data (MAKING A MESSAGE FROM SCRATCH AND NOT USING MSG_IN)
	 cv::Mat_<cv::Vec3b> _OutputBGRMat = OutputBGRMat;

     for( int y = 0; y < OutputBGRMat.rows; ++y)
        for( int x = 0; x < OutputBGRMat.cols; ++x )
           {
               _OutputBGRMat(y,x)[0] = imRef(outputIm,x,y).b;
               _OutputBGRMat(y,x)[1] = imRef(outputIm,x,y).g;
               _OutputBGRMat(y,x)[2] = imRef(outputIm,x,y).r;
        }

      OutputBGRMat= _OutputBGRMat;


	//now convert OutputMat to grayscale
	cv::cvtColor(OutputBGRMat, in_msg->image, CV_BGR2GRAY);

	cv_bridge::CvImage out_msg;
	out_msg = cv_bridge::CvImage(in_msg->header, sensor_msgs::image_encodings::BGR8, OutputBGRMat);
    //Display the image using OpenCV
    cv::imshow(WINDOW, out_msg.image);
    cv::waitKey(3);

    pub.publish(out_msg.toImageMsg());
}

 
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
    
        ros::init(argc, argv, "image_processor");
  
        ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
        image_transport::Subscriber sub = it.subscribe("/camera/depth/image", 1, imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
        pub = it.advertise("/camera/depth/segmented", 1);
        ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}


