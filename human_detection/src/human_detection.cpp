#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>       /* isnan, sqrt */
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include "segment_depth/segment.h"
#include "merge_and_filter/merge_and_filter.h"

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

//Use method of ImageTransport to create image 3 publishers
image_transport::Publisher pub;
image_transport::Publisher pubNormalIm;
image_transport::Publisher pubDepthSeg;
image_transport::Publisher pubNormalSeg;
image_transport::Publisher pubcandidates;


void publish_image(image<rgb> *image, image_transport::Publisher publisher, cv_bridge::CvImagePtr in_msg, int alpha, const char* window, bool showwindow){
	//create RGB Mat
	cv::Mat OutputBGRMat = cv::Mat::zeros(in_msg->image.size(),CV_8UC3);

	//fill in NEW mat with segmented data (MAKING A MESSAGE FROM SCRATCH AND NOT USING MSG_IN)
	 cv::Mat_<cv::Vec3b> _OutputBGRMat = OutputBGRMat;

     for( int y = 0; y < OutputBGRMat.rows; ++y){
        for( int x = 0; x < OutputBGRMat.cols; ++x )
           {
               _OutputBGRMat(y,x)[0] = imRef(image,x/alpha,y/alpha).b;
               _OutputBGRMat(y,x)[1] = imRef(image,x/alpha,y/alpha).g;
               _OutputBGRMat(y,x)[2] = imRef(image,x/alpha,y/alpha).r;
        }
	}

    OutputBGRMat= _OutputBGRMat;

	cv_bridge::CvImage out_msg;
	out_msg = cv_bridge::CvImage(in_msg->header, sensor_msgs::image_encodings::BGR8, OutputBGRMat);
	//Display the image using OpenCV
    
	if(showwindow){
		cv::imshow(window, out_msg.image);
	}

	publisher.publish(out_msg.toImageMsg());
}	



//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	//start of clock timing	
	clock_t start = clock(), diff;

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
 
	/* TODO dont need this anymore
	double minval, maxval;
    cv::minMaxIdx(in_msg->image, &minval, &maxval);
    std::cout << "Minval: " << minval << std::endl;
    std::cout << "Maxval: " << maxval << std::endl;
    //cv::Mat *output_Im = new cv::Mat();
	in_msg->image.convertTo(in_msg->image, CV_8UC1,255.0/maxval);*/

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
	float lastgoodvalue = 0;
	//std::cout << "im width: " << in_width << std::endl;
    //std::cout << "im height: " << in_height << std::endl;
	//TODO int dval = 0;	
	float dval = 0;
	//create seed for random numb generator
	srand(time(NULL));

	//subsample variables
	sub_width = in_width/alpha;
	sub_height = in_height/alpha;

	//direct conversion from Mat to image<rgb>
	image<float> *inputIm = new image<float>(sub_width,sub_height);
	image<rgb> *outputIm = new image<rgb>(sub_width,sub_height);

	//SUBSAMPLE
	//Now scan through and create a subsampled image<char> for segmenting
	for (int y=0; y < sub_height; y++){
		for(int x=0; x < sub_width; x++){ //we minus 2 as incoming images are missing their right most column
			//get s values randomly from each cell into vector
			//TODO std::vector<uchar> samples;
			std::vector<float> samples;
			for (int i = 0 ; i <= s; i++){
				//random x and y offset vals
				int dx = rand() % alpha;
				int dy = rand() % alpha;
				//grab the associated pel from the input image
				//TODO dval = in_msg->image.at<uchar>(y*alpha+dy,x*alpha+dx);
				dval = in_msg->image.at<float>(y*alpha+dy,x*alpha+dx);
				if(!isnan(dval)){
					samples.push_back(dval);
				}
			}
			if(!samples.empty()){
				//arrange the samples with the median value in the position of s/2
				std::nth_element(samples.begin(), samples.begin()+samples.size()/2, samples.end());
				//take the median value to be our sampled point
				imRef(inputIm,x,y) = samples[samples.size()/2];
				lastgoodvalue = samples[samples.size()/2];
				//std::cout << "pixel (" << x << "," << y << "): " << samples[s/2] << std::endl;
			}
			else {
				imRef(inputIm,x,y) = lastgoodvalue;
			}
		}
	}

	int num_ccs = 0;
	image<rgb> * normalIm = new image<rgb>(sub_width,sub_height); //assign memory
	image<rgb> * depthseg = new image<rgb>(sub_width,sub_height);
	image<rgb> * normalseg = new image<rgb>(sub_width,sub_height);
	
	//segment image
	//outputIm = segment_image1C(inputIm, sigma, Kdepth, Knormal , min_size, &num_ccs, &normalIm, &depthseg, &normalseg);
	universe *u_segmented = segment_image1C(inputIm, sigma, Kdepth, Knormal , min_size, &num_ccs, &normalIm, &depthseg, &normalseg, &outputIm);
	//merge regions
	std::vector<candidate> candidates = merge_and_filter(inputIm, u_segmented, sub_width, sub_height);

	//create image and fill black	
	image<rgb> * candidates_image = new image<rgb>(sub_width,sub_height);
	for(int y = 0; y < sub_height; y++) {
		for (int x = 0; x < sub_width; x++) {
		imRef(candidates_image, x, y).r = 0;
		imRef(candidates_image, x, y).g = 0;
		imRef(candidates_image, x, y).b = 0;
		}
	}
	
	//now fill in image with remaining candidates
	for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
		//it->add_to_image(candidates_image, random_rgb());
		if( !(it->erased) ){
			rgb colour = random_rgb();		
			for(std::vector<cv::Point3f>::iterator itp = it->pts.begin(); itp != it->pts.end(); itp++){
				int x = itp-> x;
				int y = itp-> y;
				imRef(candidates_image, x, y).r = colour.r;
				imRef(candidates_image, x, y).g = colour.g;
				imRef(candidates_image, x, y).b = colour.b;
			}
		}
	}




	publish_image(outputIm, pub, in_msg, alpha, WINDOW, SHOWJOINTSEG);
	publish_image(normalIm, pubNormalIm, in_msg, alpha, WNORMALIM, SHOWNORMALIM);
	publish_image(depthseg, pubDepthSeg, in_msg, alpha, WDEPTHSEG, SHOWDEPTHSEG);
	publish_image(normalseg, pubNormalSeg, in_msg, alpha, WNORMALSEG, SHOWNORMALSEG);
	publish_image(candidates_image, pubcandidates, in_msg, alpha, WCANDIDATES, SHOWCANDIDATES);
	
	if(SHOWDEPTHIM){
		//rescale for viewing
		double minval, maxval;
	    cv::minMaxIdx(in_msg->image, &minval, &maxval);
		in_msg->image.convertTo(in_msg->image, CV_8UC1,255.0/maxval);	
		cv::imshow(WDEPTHIM, in_msg->image);
	}
	cv::waitKey(1);

	//end of timing
	diff = clock() - start;
	int msec = diff * 1000 / CLOCKS_PER_SEC;
	//printf("Time taken %d seconds %d milliseconds", msec/1000, msec%1000);
	//std::cout << "Time taken to process frame: " << msec/1000 << "s " << msec%1000 << "ms " << std::endl;
	//std::cout << std::endl << "END OF FRAME " << std::endl << std::endl;

	
}

int main(int argc, char **argv)
{
    
        ros::init(argc, argv, "image_processor");
  
        ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
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
    
    image_transport::Subscriber sub = it.subscribe("/camera/depth/image", 1, imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.

	pub = it.advertise("/camera/depth/segmented", 1);
	pubNormalIm = it.advertise("/camera/depth/normal", 1);
	pubDepthSeg = it.advertise("/camera/depth/dSeg",1);
	pubNormalSeg = it.advertise("/camera/depth/nSeg",1);
	pubcandidates = it.advertise("/human_detection/candidates", 1);
// By Commenting these out our windows stay where we leave them ACTUALLY IT MAKES NO DIFFERENCE!
#if SHOWJOINTSEG
	cv::destroyWindow(WINDOW);
#endif

#ifdef SHOWNORMALIM
    cv::destroyWindow(WNORMALIM);
#endif

#ifdef SHOWDEPTHSEG
    cv::destroyWindow(WDEPTHSEG);
#endif

#ifdef SHOWNORMALSEG
	cv::destroyWindow(WNORMALSEG);
#endif

#ifdef SHOWCANDIDATES
	cv::destroyWindow(WCANDIDATES);
#endif

        ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}


