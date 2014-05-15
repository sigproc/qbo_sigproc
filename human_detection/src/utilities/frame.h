#include <stdlib.h>
#include <time.h>
#include <math.h>       /* isnan, sqrt */
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "../segment_depth/segment.h"
#include "../merge_and_filter/merge_and_filter.h"
#include "file_management.h"

#include "../config.h"

/*********************************************************************************
**************************  Current Comments  ************************************


*********************************************************************************
*********************************************************************************/

class frame {

	public:
	//for interfacing
	std::string path;
	cv::Mat depth_32FC1;
	std::vector<candidate> candidates;
	
	//for timing
	clock_t start, postPrep, postSeg, postMerge, postPost;

	//constructors
	frame();
	frame(std::string path, bool &success);
	~frame();
	void erase_ims();

	//preprocessing
	void get_candidates();
	int valid_candidates();

	//visualisation
	cv::Mat get_normalIm(bool with_cand_boxes);
	cv::Mat get_depthseg(bool with_cand_boxes);
	cv::Mat get_normalseg(bool with_cand_boxes);
	cv::Mat get_jointseg(bool with_cand_boxes);
	cv::Mat get_depthIm(bool with_cand_boxes);
	cv::Mat get_candidates_im();

	private:
	//require only for internal stuff
	image<float> * subsampled; //was inputim
	//get_candidates() params
	float sigma, Kdepth, Knormal;
	int min_size, alpha, s;

	universe * u_segmented;

	image<rgb> * normalIm;
	image<rgb> * depthseg;
	image<rgb> * normalseg;
	image<rgb> * jointseg; //was outputIm

	//internal methods
	cv::Mat rgbIm_to_Mat(image<rgb> * rgbIm, bool with_cand_boxes);

};

//constructors
frame::frame(){
}
frame::frame(std::string path, bool &success){

	sigma = SIGMA;
	Kdepth = KDEPTH;
	Knormal = KNORMAL;
	min_size = MIN_SIZE;
	alpha = ALPHA, s = ESS;

	//read in and verify that data is valid
	readFileToMat(depth_32FC1, path);
	if (!depth_32FC1.data){
		std::cout << "Error : Path " << path << " cannot be loaded!!" << std::endl;
		success = false;
		return;
	}

	success = true;
}
//desctructer
frame::~frame(){
	
}


/**************************************************************

	get_candidates(): Runs segmentation and merging on
					  depth Mat to get candidates

***************************************************************/
void frame::get_candidates(){

	start = clock();

	//params for conversion
	int in_width, in_height;
	int sub_width, sub_height;
	in_width = depth_32FC1.size().width;
	in_height = depth_32FC1.size().height;
	float lastgoodvalue = 0;	
	float dval = 0;
	//create seed for random numb generator
	srand(time(NULL));

	//subsample variables
	sub_width = in_width/alpha;
	sub_height = in_height/alpha;

	//direct conversion from Mat to image<rgb>
	subsampled = new image<float>(sub_width,sub_height);

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
				dval = depth_32FC1.at<float>(y*alpha+dy,x*alpha+dx);
				if(!isnan(dval)){
					samples.push_back(dval);
				}
			}
			if(!samples.empty()){
				//arrange the samples with the median value in the position of s/2
				std::nth_element(samples.begin(), samples.begin()+samples.size()/2, samples.end());
				//take the median value to be our sampled point
				imRef(subsampled,x,y) = samples[samples.size()/2];
				lastgoodvalue = samples[samples.size()/2];
			}
			else {
				imRef(subsampled,x,y) = lastgoodvalue;
			}
		}
	}

	int num_ccs = 0;

	postPrep = clock();	

	//segment image
	u_segmented = segment_image1C(subsampled, sigma, Kdepth, Knormal, min_size, &num_ccs, &normalIm, &depthseg, &normalseg, &jointseg);

	postSeg = clock();

	//merge regions
	candidates = merge_and_filter(subsampled, u_segmented, sub_width, sub_height, depth_32FC1);

	postMerge = clock();
}


int frame::valid_candidates(){

	int count = 0;

	for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
		if( !(it->erased) ) count++;
	}

	return count;
}

void frame::erase_ims(){
	if(!candidates.empty()){
		//Clean up image containers
		delete subsampled;
		delete jointseg;
		delete normalIm;
		delete depthseg;
		delete normalseg;
		delete u_segmented;
		//std::cout << "frames memory deleted" << std::endl;
	}
}
/**************************************************************

						visualisations

***************************************************************/
cv::Mat frame::get_normalIm(bool with_cand_boxes){
	return rgbIm_to_Mat(normalIm, with_cand_boxes);
}
cv::Mat frame::get_depthseg(bool with_cand_boxes){
	return rgbIm_to_Mat(depthseg, with_cand_boxes);
}
cv::Mat frame::get_normalseg(bool with_cand_boxes){
	return rgbIm_to_Mat(normalseg, with_cand_boxes);
}
cv::Mat frame::get_jointseg(bool with_cand_boxes){
	return rgbIm_to_Mat(jointseg, with_cand_boxes);
}
cv::Mat frame::get_depthIm(bool with_cand_boxes){

	cv::Mat output;
	double minval, maxval;
	cv::minMaxIdx(depth_32FC1, &minval, &maxval);
	depth_32FC1.convertTo(output, CV_8UC1,255.0/maxval);
	cv::cvtColor(output, output, CV_GRAY2BGR);

	if(with_cand_boxes){
		if(!candidates.empty()){	
			for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
				if( !(it->erased) ){
					cv::rectangle(output, it->boundingBox, cv::Scalar(0,0,255), 2);
				}
			}
		}
		else {
			std::cout << "Cannot show candidates and they have not been found" << std::endl;
		}
	}

	return output;
}

cv::Mat frame::get_candidates_im(){

	int in_width, in_height;
	int sub_width, sub_height;
	in_width = depth_32FC1.size().width;
	in_height = depth_32FC1.size().height;

	//subsample variables
	sub_width = in_width/alpha;
	sub_height = in_height/alpha;

	image<rgb> * candidates_image = new image<rgb>(sub_width,sub_height);
	
	for(int y = 0; y < sub_height; y++) {
		for (int x = 0; x < sub_width; x++) {
		imRef(candidates_image, x, y).r = 0;
		imRef(candidates_image, x, y).g = 0;
		imRef(candidates_image, x, y).b = 0;
		}
	}

	for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
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

	bool with_cand_boxes = false;

	cv::Mat candidates_im = rgbIm_to_Mat(candidates_image, with_cand_boxes);
	delete candidates_image;

	return candidates_im;
}

cv::Mat frame::rgbIm_to_Mat(image<rgb> * rgbIm, bool with_cand_boxes){
	//create Mat container	
	cv::Mat OutputBGRMat = cv::Mat::zeros(depth_32FC1.size(),CV_8UC3);

	//fill in NEW mat with segmented data (MAKING A MESSAGE FROM SCRATCH AND NOT USING MSG_IN)
	 cv::Mat_<cv::Vec3b> _OutputBGRMat = OutputBGRMat;

     for( int y = 0; y < OutputBGRMat.rows; ++y){
        for( int x = 0; x < OutputBGRMat.cols; ++x )
           {
               _OutputBGRMat(y,x)[0] = imRef(rgbIm,x/alpha,y/alpha).b;
               _OutputBGRMat(y,x)[1] = imRef(rgbIm,x/alpha,y/alpha).g;
               _OutputBGRMat(y,x)[2] = imRef(rgbIm,x/alpha,y/alpha).r;
        }
	}

    OutputBGRMat= _OutputBGRMat;

	if(with_cand_boxes){
		if(!candidates.empty()){	
			for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
				if( !(it->erased) ){
					cv::rectangle(OutputBGRMat, it->boundingBox, cv::Scalar(0,0,255), 2);
				}
			}
		}
		else {
			std::cout << "Cannot show candidates and they have not been found" << std::endl;
		}
	}
	
	return OutputBGRMat;
}
