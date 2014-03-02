#include <stdlib.h>
#include <time.h>
#include <math.h>       /* isnan, sqrt */
#include <vector>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include "segment_depth/segment.h"
#include "merge_and_filter/merge_and_filter.h"
#include "classification/descriptor.h"
#include "utilities.h"


#include "config.h"


/*********************************************************************************
**************************  Current Comments  ************************************






*********************************************************************************
*********************************************************************************/
 
//prototype
int segment(float sigma, float k, int min_size, cv::Mat image);

class train_frame {

	public: 
	std::string im_filename;
	std::vector<candidate> candidates;
	std::ofstream * boxes;
	std::ofstream * descriptors;
	cv::Mat frame;

	train_frame();
	~train_frame();
	train_frame(std::string im_filename_, cv::Mat frame_, std::ofstream * boxes_, std::ofstream * descriptors_);
	void tag_frame();
	void tag_candidates();

};

//constructor
train_frame::train_frame(){
}

train_frame::train_frame(std::string im_filename_, cv::Mat frame_, std::ofstream * boxes_, std::ofstream * descriptors_){
	boxes = boxes_;
	descriptors = descriptors_;
	frame = frame_;
	im_filename = im_filename_;
}
	
//destructor
train_frame::~train_frame(){
}

//The beef!
void train_frame::tag_frame(){
	*boxes << im_filename << " tlx tly width height visibility" << std::endl;
}

void train_frame::tag_candidates(){

	*descriptors << '+' << '1' << ' ';
	for (int i = 0; i < 20 ; i++){
		*descriptors << i << ':' << i << '.' << i << ' ';
	}
	*descriptors << '#' << std::endl;
}


int main ( int argc, char **argv){

	//THE ARGUMENT WILL BE THAT OF THE DIRECTORY WHICH CONTAINS THE IMAGES
	const char* bag_cstr = argv[1];
	bool tag_frames = true;
	bool tag_candidates = true;

	if (bag_cstr == NULL){
		std::cout << "No Image path given in execution, loading from default" << std::endl;
		bag_cstr = "bag_frames/trash/";
	}

	std::string bag = bag_cstr;
	std::string::iterator last_char = bag.end();
	last_char--;

	//std::cout << "last char: " << *last_char << std::endl;
	
	if (*last_char != '/'){
		bag.append("/");
		std::cout << "Added '/' to end of dir" << std::endl;
	}

	std::string path = FRAMESDIR;
	path.append(bag);

	std::cout << "Training from dir: " << path << std::endl;
	
	DIR *dir;
	struct dirent *ent;
	std::list<std::string> frames;
	if ((dir = opendir (path.c_str())) != NULL) {
	  // print all the files and directories within directory 
	  while ((ent = readdir (dir)) != NULL) {
		frames.push_back(ent->d_name);
	  }
	  closedir (dir);
	} else {
	  // could not open directory 
	  perror ("");
	  return EXIT_FAILURE;
	}

	//sort into numerical order
	frames.sort();
	frames.remove(".");
	frames.remove("..");
	
	std::list<std::string>::iterator im_file;
	//and print out
	for(im_file = frames.begin(); im_file != frames.end(); im_file++){
		std::cout << " " << *im_file << std::endl;
	}
	std::cout << std::endl;

	//Open Frame  and candidate annotation files
	std::string anttn_frame_fname = ANNOTDIR;
	anttn_frame_fname.append(bag);
	anttn_frame_fname.append("boxes");
	
	std::string anttn_cand_fname = ANNOTDIR;
	anttn_cand_fname.append(bag);
	anttn_cand_fname.append("descriptors");

	std::ofstream anttn_frame_f;
	anttn_frame_f.open(anttn_frame_fname.c_str());
	std::ofstream anttn_cand_f;
	anttn_cand_f.open(anttn_cand_fname.c_str());

	for(im_file = frames.begin(); im_file != frames.end(); im_file++){
		//Get each mat and display
		std::string f_dir = path + *im_file;
		//std::cout << f_dir << std::endl;
		cv::Mat inframe;
		//read in and verify that data is valid
		readFileToMat(inframe, f_dir);
		if (!inframe.data){
			std::cout << "Error : Path " << f_dir << " cannot be loaded!!" << std::endl;
		}
		else{
			cv::imshow("WINDOW", inframe);
			cv::waitKey(100);


			train_frame f = train_frame(*im_file, inframe, &anttn_frame_f, &anttn_cand_f);

			if(tag_frames){
				f.tag_frame();
			}
			if(tag_candidates){
				f.tag_candidates();
			}

		}
	}

return 0;
}













/*//This function is called everytime a new image is published
void Human_Detector::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
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
	
	cv::waitKey(1000);

	//Open File to write descriptors to. Contents of this file are re-written for each frame!
	std::ofstream file;
	std::string datadir = DATADIR;
	datadir.append("test.txt");
	file.open (datadir.c_str());

	int num_candidates = 0;

	//show/evaluate candidates
	for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
		if( !(it->erased) ){
			descriptor candidate_descriptor = descriptor(it->im);
			candidate_descriptor.compute_descriptor();
			candidate_descriptor.check_human();
			cv::Mat display;
			it->im.convertTo(display, CV_8UC1,255.0/maxval);
			cv::Mat histograms = candidate_descriptor.visualise_cells(4);
			cv::Mat mags = candidate_descriptor.visualise_gmags(4);
			cv::Mat dirs = candidate_descriptor.visualise_gdirs(4);
			//convert gradients to 3 channel char
			double gmin, gmax;
			cv::minMaxIdx(mags, &gmin, &gmax);
			cv::cvtColor(mags,mags, CV_GRAY2BGR);
			mags.convertTo(mags, CV_8UC3,255.0/gmax);
			//std::cout << "Histogram size: " << histograms.size() << ", type: " << histograms.type() << std::endl;
			//std::cout << "Gradients size: " << mags.size() << ", type: " << mags.type() << std::endl;
			//gradients = gradients+histograms;
			cv::addWeighted(mags,1,histograms,1,0,histograms,-1);
			cv::addWeighted(mags,1,dirs,1,0,dirs,-1);
			cv::imshow("Current Candidate", display);
			cv::imshow("Candidate Cell Magnitudes", dirs);
			cv::imshow("Candidate Cell Histograms", histograms);

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
	delete candidates_image;

	//clean up other containers
	delete u_segmented;

	
	//end of timing
//	int msec = diff * 1000 / CLOCKS_PER_SEC;
	int pre_ms = (postPrep - start) * 1000/ CLOCKS_PER_SEC;
	int Seg_ms = (postSeg - postPrep) * 1000 / CLOCKS_PER_SEC;
	int Merge_ms = (postMerge -postSeg) * 1000 / CLOCKS_PER_SEC;
	
	std::cout << std::endl << "END OF FRAME " << std::endl << std::endl;

	
}


*/

/********************************************************

*******************************************************/


