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
static const char WINDOW[] = "Stream";
 
//prototype
int segment(float sigma, float k, int min_size, cv::Mat image);

/**********************************************8


***********************************************/

class Human_Detector
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber sub_;
	//Use method of ImageTransport to create all publishers
	image_transport::Publisher pub_;

public:
  Human_Detector()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    sub_ =it_.subscribe("/camera/depth/image", 1, &Human_Detector::imageCallback, this);

	pub_ = it_.advertise("/human_detection/segmentation", 1);
	

    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
  }

  ~Human_Detector()
  {
	cv::destroyAllWindows();
  }

	void imageCallback(const sensor_msgs::ImageConstPtr& original_image);
};

/***************************************************************

*********************************************************************/
//This function is called everytime a new image is published
void Human_Detector::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	//start of clock timing	
	clock_t start = clock(), postPrep, postSeg, postMerge, postClass, postPost;

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

	//display image
	cv::Mat im_out = in_msg->image.clone();
	double minval, maxval;
	cv::minMaxIdx(im_out, &minval, &maxval);
	//rescale for viewing
	im_out.convertTo(im_out, CV_8UC1,255.0/maxval);
	cv::cvtColor(im_out, im_out, CV_GRAY2BGR);
	//cv::imshow(WINDOW, im_out);
	//cv::waitKey(3);
 
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
			std::vector<float> samples;
			for (int i = 0 ; i <= s; i++){
				//random x and y offset vals
				int dx = rand() % alpha;
				int dy = rand() % alpha;
				//grab the associated pel from the input image
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
	universe *u_segmented = segment_image1C(inputIm, sigma, Kdepth, Knormal , min_size, &num_ccs, &normalIm, &depthseg, &normalseg, &outputIm);

	postSeg = clock();

	//merge regions
	std::vector<candidate> candidates = merge_and_filter(inputIm, u_segmented, sub_width, sub_height, in_msg->image);

	postMerge = clock();

	//Open File to write descriptors to. Contents of this file are re-written for each frame!
	std::ofstream file;
	std::string datadir = DATADIR;
	datadir.append("test.txt");
	file.open (datadir.c_str());

	int num_candidates = 0;
	std::list<cv::Rect> boundingBoxes;

	//show/evaluate candidates
	for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
		if( !(it->erased) ){
			descriptor candidate_descriptor = descriptor(it->im);
			candidate_descriptor.compute_descriptor();
			boundingBoxes.push_back(it->boundingBox);
			//print candidates to file
			file << candidate_descriptor.HODstring();
			cv::waitKey(1000);
			num_candidates++;
		}
	}

	//close file
	file.close();

	//Clean up image containers
	delete inputIm;
	delete outputIm;
	delete normalIm;
	delete depthseg;
	delete normalseg;

	//clean up other containers
	delete u_segmented;

	//skip the following if there are no candidates
	if( num_candidates > 0){
		//Now run classifier on file
		std::string outputfile = DATADIR;
		outputfile.append("mock/predictions");

		std::string model = MODEL;
		std::string fncall = SVMDIR;
		fncall.append("svm_classify " + datadir +" " + model + " " + outputfile );

		//std::cout << fncall.c_str() << std::endl;
		std::cout << "Running classifier..." << std::endl;
		int returncode = system(fncall.c_str());

		//Get boundingboxes iterator
		std::list<cv::Rect>::iterator it = boundingBoxes.begin();

		//Now interpret classifier results and draw boxes on image
		std::ifstream c_descriptors;
		c_descriptors.open(outputfile.c_str());
		for(int i = 0; i < num_candidates; i++){
			char classchar[100];
			c_descriptors.getline(classchar,100);
			std::cout << "Read Line: " << classchar;
			float classval = atof(classchar);
			std::cout << " has value: " << classval << std::endl;

			cv::Rect cand_box = *it++;
		
			if (classval > 0){
				std::cout << "Candidate: " << i << " is a human!" << std::endl;
				cv::rectangle(im_out,cand_box, cv::Scalar(0,0,255), 2);
			}
			else {
				cv::rectangle(im_out,cand_box, cv::Scalar(255,255,255), 2);
			}
		}

		cv::imshow(WINDOW, im_out);
		c_descriptors.close();
	}
	postClass = clock();

	//end of timing
//	int msec = diff * 1000 / CLOCKS_PER_SEC;
	int pre_ms = (postPrep - start) * 1000/ CLOCKS_PER_SEC;
	int Seg_ms = (postSeg - postPrep) * 1000 / CLOCKS_PER_SEC;
	int Merge_ms = (postMerge -postSeg) * 1000 / CLOCKS_PER_SEC;
	int Classif_ms = (postClass - postMerge) * 1000 / CLOCKS_PER_SEC;

	std::cout << "Time taken to process each frame: " << std::endl;
	std::cout << "Pre Processing: " << pre_ms/1000 << "s " << pre_ms%1000 << "ms " << std::endl;
	std::cout << "Seg Processing: " << Seg_ms/1000 << "s " << Seg_ms%1000 << "ms " << std::endl;
	std::cout << "Merge Processing: " << Merge_ms/1000 << "s " << Merge_ms%1000 << "ms " << std::endl;
	std::cout << "Classification Processing: " << Classif_ms/1000 << "s " << Classif_ms%1000 << "ms " << std::endl;

	postPost = clock();
	int post_ms = (postPost - postClass)* 1000 / CLOCKS_PER_SEC;
	std::cout << "Post Processing: " << post_ms/1000 << "s " << post_ms%1000 << "ms " << std::endl;

	int total_time = (postPost - start)	* 1000/CLOCKS_PER_SEC;
	std::cout << "Total Time: " << total_time << std::endl;	

	std::cout << std::endl << "END OF FRAME " << std::endl << std::endl;

	

	//wait to smooth things out
	/*if (total_time < 500){
		cv::waitKey( 500 - total_time);
	}*/

	
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

