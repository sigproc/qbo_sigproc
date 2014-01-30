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
    sub_ =it_.subscribe("/camera/depth/image", 1, &Human_Detector::imageCallback, this);

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

	void publish_image(image<rgb> *image, image_transport::Publisher publisher, cv_bridge::CvImagePtr in_msg, int alpha, const char* window, bool showwindow,std::list<cv::Rect> boundingBoxes);

	void imageCallback(const sensor_msgs::ImageConstPtr& original_image);
	

};

void Human_Detector::publish_image(image<rgb> *image, image_transport::Publisher publisher, cv_bridge::CvImagePtr in_msg, int alpha, const char* window, bool showwindow, std::list<cv::Rect> boundingBoxes){
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
#if SHOWBOXES
	for(std::list<cv::Rect>::iterator it = boundingBoxes.begin(); it != boundingBoxes.end(); it++){
		cv::rectangle(OutputBGRMat, *it, cv::Scalar(0,0,255), 2);
	}
#endif
	cv_bridge::CvImage out_msg;
	out_msg = cv_bridge::CvImage(in_msg->header, sensor_msgs::image_encodings::BGR8, OutputBGRMat);
	//Display the image using OpenCV
    
	if(showwindow){
		cv::imshow(window, out_msg.image);
	}

	publisher.publish(out_msg.toImageMsg());
}

/***************************************************************

*********************************************************************/
//This function is called everytime a new image is published
void Human_Detector::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	//start of clock timing	
	clock_t start = clock(), postPrep, postSeg, postMerge, postPost;

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
	image<rgb> * normalIm;
	image<rgb> * depthseg;
	image<rgb> * normalseg;
	image<rgb> *outputIm;
	
	postPrep = clock();

	//segment image
	//outputIm = segment_image1C(inputIm, sigma, Kdepth, Knormal , min_size, &num_ccs, &normalIm, &depthseg, &normalseg);
	universe *u_segmented = segment_image1C(inputIm, sigma, Kdepth, Knormal , min_size, &num_ccs, &normalIm, &depthseg, &normalseg, &outputIm);

	postSeg = clock();

	//merge regions
	std::vector<candidate> candidates = merge_and_filter(inputIm, u_segmented, sub_width, sub_height, in_msg->image);

	postMerge = clock();

	//create image and fill black	
	image<rgb> * candidates_image = new image<rgb>(sub_width,sub_height);
	for(int y = 0; y < sub_height; y++) {
		for (int x = 0; x < sub_width; x++) {
		imRef(candidates_image, x, y).r = 0;
		imRef(candidates_image, x, y).g = 0;
		imRef(candidates_image, x, y).b = 0;
		}
	}
	
	std::list<cv::Rect> boundingBoxes;
	
	//now fill in image with remaining candidates and calc
	for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
		if( !(it->erased) ){
			boundingBoxes.push_back(it->boundingBox);
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
	//display images before showing candidates
	double minval, maxval;
	if(SHOWDEPTHIM){
		//rescale for viewing
	    cv::minMaxIdx(in_msg->image, &minval, &maxval);
		in_msg->image.convertTo(in_msg->image, CV_8UC1,255.0/maxval);
#if SHOWBOXES
		for(std::list<cv::Rect>::iterator it = boundingBoxes.begin(); it != boundingBoxes.end(); it++){
			cv::rectangle(in_msg->image, *it, cv::Scalar(0,0,255), 2);
		}
#endif
		cv::imshow(WDEPTHIM, in_msg->image);
	}
	if( SHOWDEPTHIM ||SHOWDEPTHSEG || SHOWNORMALSEG || SHOWNORMALIM || SHOWJOINTSEG || SHOWCANDIDATES ){
		cv::waitKey(10);
	}
	
	publish_image(outputIm, pub_, in_msg, alpha, WINDOW, SHOWJOINTSEG, boundingBoxes);
	publish_image(normalIm, pubNormalIm_, in_msg, alpha, WNORMALIM, SHOWNORMALIM, boundingBoxes);
	publish_image(depthseg, pubDepthSeg_, in_msg, alpha, WDEPTHSEG, SHOWDEPTHSEG, boundingBoxes);
	publish_image(normalseg, pubNormalSeg_, in_msg, alpha, WNORMALSEG, SHOWNORMALSEG, boundingBoxes);
	publish_image(candidates_image, pubcandidates_, in_msg, alpha, WCANDIDATES, SHOWCANDIDATES, boundingBoxes);

	cv::waitKey(1000);


	//show candidates
	for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
		if( !(it->erased) ){
			descriptor candidate_descriptor = descriptor(it->im);
			candidate_descriptor.compute_descriptor();
			candidate_descriptor.check_human();
			cv::Mat display;
			/*cv::Mat display_cells(it->im.size(),CV_8UC1);
			for(int x = 0; x < display_cells.cols; x++){
				for(int y = 0; y < display_cells.rows; y++){
					display_cells.at<uchar>(cv::Point(x,y)) = candidate_descriptor.grad_mag.at<float>(cv::Point(x/16,y/16));
				}
			}
			double maxval2;
			cv::minMaxIdx(candidate_descriptor.grad_mag, &minval, &maxval2);*/
			it->im.convertTo(display, CV_8UC1,255.0/maxval);
			cv::imshow("Current Candidate", display);
			cv::imshow("Candidate Cell Magnitudes", candidate_descriptor.grad_mag);
			cv::waitKey(1000);
		}
	}

	//Clean up image containers
	delete inputIm;
	delete outputIm;
	delete normalIm;
	delete depthseg;
	delete normalseg;
	delete candidates_image;

	//clean up other containers
	delete u_segmented;

	//end of timing
//	int msec = diff * 1000 / CLOCKS_PER_SEC;
	int pre_ms = (postPrep - start) * 1000/ CLOCKS_PER_SEC;
	int Seg_ms = (postSeg - postPrep) * 1000 / CLOCKS_PER_SEC;
	int Merge_ms = (postMerge -postSeg) * 1000 / CLOCKS_PER_SEC;

	/*std::cout << "Time taken to process each frame: " << std::endl;
	std::cout << "Pre Processing: " << pre_ms/1000 << "s " << pre_ms%1000 << "ms " << std::endl;
	std::cout << "Seg Processing: " << Seg_ms/1000 << "s " << Seg_ms%1000 << "ms " << std::endl;
	std::cout << "Merge Processing: " << Merge_ms/1000 << "s " << Merge_ms%1000 << "ms " << std::endl;

	postPost = clock();
	int post_ms = (postPost - postMerge)* 1000 / CLOCKS_PER_SEC;
	std::cout << "Post Processing: " << post_ms/1000 << "s " << post_ms%1000 << "ms " << std::endl;*/

	std::cout << std::endl << "END OF FRAME " << std::endl << std::endl;

	
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

