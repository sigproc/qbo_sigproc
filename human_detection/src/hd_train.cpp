#include "config.h"

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





/*********************************************************************************
**************************  Current Comments  ************************************






*********************************************************************************
*********************************************************************************/
 
//prototypes and definitions
int segment(float sigma, float k, int min_size, cv::Mat image);

enum uiState_t { drawn, drawing, neutral };

class train_frame {

	public: 
	std::string im_filename;
	std::vector<candidate> candidates;
	std::ofstream * s_boxes;
	std::ofstream * s_descriptors;
	std::ofstream * s_log;
	cv::Mat frame;

	train_frame();
	~train_frame();
	train_frame(std::string im_filename_, cv::Mat frame_, std::ofstream * s_boxes_, std::ofstream * s_descriptors_, std::ofstream * s_log_);
	void tag_frame(bool * exit);
	void tag_candidates();
	void save_session(std::string bag, std::string session, std::string f_start);

	
	cv::Point mouse;
	cv::Point box_start;
	cv::Point box_end;
	bool draw;
	uiState_t s;

	void draw_rectangle(cv::Point start, cv::Point end, const cv::Scalar& color, cv::Mat frame);

};

void onMouse_frame( int event, int x, int y, int, void*);
void onMouse_cand( int event, int x, int y, int, void*);
void dummy( int event, int x, int y, int, void*);

void classify_candidates(std::vector<candidate> candidates, std::ofstream * descriptors, std::ofstream * log);
std::vector<candidate> get_candidates(cv::Mat frame);


/***********************************************************************

							train_frame methods


**********************************************************************/
//constructor
train_frame::train_frame(){
}

train_frame::train_frame(std::string im_filename_, cv::Mat frame_, std::ofstream * s_boxes_, std::ofstream * s_descriptors_, std::ofstream * s_log_){
	s_boxes = s_boxes_;
	s_descriptors = s_descriptors_;
	s_log = s_log_;
	frame = frame_;
	im_filename = im_filename_;
}
	
//destructor
train_frame::~train_frame(){
}

//The beef!
void train_frame::tag_frame(bool * exit){

	s = neutral;
	box_start = cv::Point(0,0);
	box_end = cv::Point(0,0);
	mouse = cv::Point(0,0);

	//create window
	cv::namedWindow("tag_frame", CV_WINDOW_AUTOSIZE);
	double minval, maxval;

	//show frame in better contrast
	cv::Mat img = frame.clone();
	cv::minMaxIdx(img, &minval, &maxval);
	img.convertTo(img, CV_8UC1,255.0/maxval);
	cv::imshow("tag_frame", img);

	//initialise params

	//Set up the callbacks
	cv::setMouseCallback("tag_frame",onMouse_frame, (void*)this);
	//infinite loop until we exit or finish
	char oldc = NULL;
	int count = 0;
	while( 1 ) {

		if(s == drawing){
			draw_rectangle(box_start, mouse, CV_RGB(255,0,0), frame);
		}
		else if(s == drawn){
			draw_rectangle(box_start, box_end, CV_RGB(255,0,0), frame);
			s = neutral;
		}
		
		char c = (char)cv::waitKey(15);
		char vis;
		//if( c != oldc){
		//	std::cout << (int)c << std::endl;
		//}

		if (c == 32){
			//on a space leave while loop
			*s_log << im_filename << " boxes:" << count;
			break;
		}
		switch(c){
			//Do actions without leaving while loop and log records
			case 'h':
				vis = (int)-1;
				while(vis == -1){
					vis = (char)cv::waitKey(15);
				}
				vis = vis - '0';
				if( (vis < 0) || (vis) > 2 ){
					std::cout << "Valid visibilities are: " << std::endl;
					std::cout << "\t\t 0 = hidden" << std::endl;
					std::cout << "\t\t 1 = fully visible" << std::endl;
					std::cout << "\t\t 2 = partially visible" << std::endl;
					while ( (vis < 0) || (vis) > 2 ){
						vis = (char)cv::waitKey(15);
						vis = vis - '0';
					}
				}
				std::cout << im_filename << " ";
				std::cout << box_start.x << " " << box_start.y << " ";
				std::cout << box_end.x - box_start.x << " " << box_end.y - box_start.y << " ";	
				std::cout << (int)vis << std::endl;

				*s_boxes << im_filename << " ";
				*s_boxes << box_start.x << " " << box_start.y << " ";
				*s_boxes << box_end.x - box_start.x << " " << box_end.y - box_start.y << " ";	
				*s_boxes << (int)vis << std::endl;
				//break; This is deliberatly ignored so that after a 'hold' there is a reset
				count++;
			case 'r':
				box_start = cv::Point(0,0);
				box_end = cv::Point(0,0);
				mouse = cv::Point(0,0);
				cv::imshow("tag_frame", img);
				s = neutral;
				break;
			case 27:
				*exit = true;
				*s_log << im_filename << " boxes:" << count;
				return;
				break;
			default:
				;
				break;
		}
		oldc = c;
	}

	cv::setMouseCallback("tag_frame",dummy);
	cv::destroyWindow("tag_frame");

}

void train_frame::save_session(std::string bag, std::string session, std::string f_start){

	//read in desc_start and desc_end
	int desc_start = 0;
	int desc_end = 0;
	std::string line;

	std::cout << "Copying data from session to 'complete' files..." << std::endl;
	//close session files
	s_boxes->close();
	s_descriptors->close();
	s_log->close();
	
	//determine strings of 'complete' files
	std::string path_comp_boxes = ANNOTDIR;
	path_comp_boxes.append(bag);
	path_comp_boxes.append(COMPBOXES);
	
	std::string path_comp_desc = ANNOTDIR;
	path_comp_desc.append(bag);
	path_comp_desc.append(COMPDESC);

	std::string path_comp_log = ANNOTDIR;
	path_comp_log.append(bag);
	path_comp_log.append(COMPLOG);

	//count how many lines are in c_desc, into desc_start
	std::ifstream c_desc_i;
	c_desc_i.open(path_comp_desc.c_str(), std::ios::app);
	if(c_desc_i.is_open() ){
		while ( std::getline(c_desc_i,line) ){
			desc_start++;
		}
		c_desc_i.close();
	}

	//open these files at the end
	std::ofstream c_boxes;
	c_boxes.open(path_comp_boxes.c_str(), std::ios::app);
	std::ofstream c_log;
	c_log.open(path_comp_log.c_str(), std::ios::app);
	std::ofstream c_desc_o;
	c_desc_o.open(path_comp_desc.c_str(), std::ios::app);

	//Re-open session files as inputs
	std::string path_session = ANNOTDIR;
	path_session.append(bag + "session/" + session + "/");
	std::ifstream s_boxes((path_session + "boxes").c_str());
	std::ifstream s_desc((path_session + "descriptors").c_str());
	
	//Copy contents across
	if(s_boxes.is_open() ){
		while ( std::getline (s_boxes,line) ){
		  c_boxes << line << '\n';
		}
		s_boxes.close();
	}
	else{
		std::cout << "ERROR: Could not open session/boxes for copying to complete/boxes" << std::endl;
	}


	if(s_desc.is_open() ){
		desc_end = desc_start;
		while ( std::getline (s_desc,line) ){
			c_desc_o << line << '\n';
			desc_end++;
		}
		s_desc.close();
	}
	else {
		std::cout << "ERROR: Could not open session/descriptors for copying to complete/descriptors" << std::endl;
	}

	//Update Log
	c_log << session << " " << desc_start << " " << desc_end << " " << f_start << " " << im_filename << std::endl;

	std::cout << "Exiting" << std::endl;
	//Close remaining files
	c_log.close();
	c_boxes.close();
	c_desc_o.close();
}

void train_frame::tag_candidates(){

	std::vector<candidate> candidates = get_candidates(frame);
	classify_candidates(candidates, s_descriptors, s_log);
}


/***********************************************************************

							misc ui methods


**********************************************************************/

void onMouse_frame( int event, int x, int y, int flag, void* obj_){
	train_frame* obj = (train_frame*)obj_; 
	switch( event ) {
		case CV_EVENT_MOUSEMOVE:
				//update mouse co-ords with mouse position
				obj->mouse = cv::Point(x,y);
			break;
		case CV_EVENT_LBUTTONDOWN:
				//Start drawing the rectangle
				obj->box_start = cv::Point(x,y);
				obj->s = drawing;
			break;
		case CV_EVENT_LBUTTONUP: 
				//stop drawing the rectangle and temporarily save the box
				obj->box_end = cv::Point(x,y);
				obj->s = neutral;
			break;

		default:
			break;
		}
}

void onMouse_cand( int event, int x, int y, int flag, void* classification_){
	
	int* classification = (int*)classification_;
	switch( event ) {
		case CV_EVENT_LBUTTONDOWN:
				//This means positive so set class as +1
				*classification = +1;
			break;
		case CV_EVENT_RBUTTONDOWN: 
				//This means negative so set class as -1
				*classification = -1;
			break;

		default:
			break;
		}
}

void train_frame::draw_rectangle(cv::Point start, cv::Point end, const cv::Scalar& color, cv::Mat frame){
		cv::Mat img = frame.clone();
		int line_thickness = 2;
		double minval, maxval;

		//rescale for viewing
	    cv::minMaxIdx(img, &minval, &maxval);
		img.convertTo(img, CV_8UC1,255.0/maxval);

		//draw rec on image and show
		cv::rectangle(img, start, end, color , line_thickness);
		imshow("tag_frame", img);
}

void dummy(int event, int x, int y, int, void*){
	//This is a shell of a callback function used to 'turn off' the
	//callback 'drawline_callback_wrapper'
}

/***************************************************************************************

										Main

***************************************************************************************/
int main ( int argc, char **argv){

	//The argument is the name of the bag file.
	const char* bag_cstr = argv[1];
	bool tag_frames = true;
	bool tag_candidates = true;

	//check that a data session is given
	if (bag_cstr == NULL){
		std::cout << "ERROR: No argument given, please pass the name of the bag file to be trained." << std::endl;
		return EXIT_FAILURE;
	}

	//from data session, find frames directory and the frames within it
	std::string bag = bag_cstr;
	std::string::iterator last_char = bag.end();
	last_char--;
	
	if (*last_char != '/'){
		bag.append("/");
		//std::cout << "Added '/' to end of dir" << std::endl;
	}

	std::string path = FRAMESDIR;
	path.append(bag);
	
	DIR *dir;
	struct dirent *ent;
	std::list<std::string> frames;
	if ((dir = opendir (path.c_str())) != NULL) {
		// print all the files and directories within directory 
		while ((ent = readdir (dir)) != NULL) {
			frames.push_back(ent->d_name);
		}
		closedir (dir);
	} 
	else {
		// could not open directory 
		perror ("");
		std::cout << "ERROR: " << path << " does not exist. " << std::endl;
		return EXIT_FAILURE;
	}

	//sort into numerical order
	frames.sort();
	frames.remove(".");
	frames.remove("..");

	//check that it is not empty
	if(frames.empty()){
		std::cout << "ERROR: " << path << " contains no frames. " <<std::endl;
		return EXIT_FAILURE;
	}

	//Generate names of required directories and check that they exist
	std::string path_anttn_bag = ANNOTDIR;
	path_anttn_bag.append(bag);

	int	sit = check_make_dir(path_anttn_bag);

	if(	sit == 1){
		//we need to create the sub trees for the new folder
		if(!create_new_bag_tree(bag)){
			std::cout << "ERROR: Path " << path_anttn_bag << " does not exist and could not be created." << std::endl;
			return EXIT_FAILURE;			
		}
		std::cout << "New annotations directory created." << std::endl;
	}
	else if(sit == -1){
		std::cout << "ERROR: Path " << path_anttn_bag << " does not exist and could not be created." << std::endl;
		return EXIT_FAILURE;
	}

	//Create new session directory, and get names of all files which are to be opened
	std::string s_tag;
	create_new_session(bag, &s_tag);

	std::string path_session_boxes = ANNOTDIR;
	path_session_boxes.append(bag + "session/" + s_tag + "/boxes");
	
	std::string path_session_desc = ANNOTDIR;
	path_session_desc.append(bag + "session/" + s_tag + "/descriptors");

	std::string path_session_log = ANNOTDIR;
	path_session_log.append(bag + "session/" + s_tag + "/log.txt");

	std::string path_comp_log = ANNOTDIR;
	path_comp_log.append(bag);
	path_comp_log.append(COMPLOG);

	//Open session files
	std::ofstream s_boxes;
	s_boxes.open(path_session_boxes.c_str());
	std::ofstream s_descriptors;
	s_descriptors.open(path_session_desc.c_str());
	std::ofstream s_log;
	s_log.open(path_session_log.c_str());

	//Open complete log file to find last entry
	std::ifstream c_log((path_comp_log.c_str()) );
	//get to last line
	std::string line, last_line;
	if(c_log.is_open() ){
		while ( std::getline (c_log,line) ){
			last_line = line;
		}
		c_log.close();
	}
	else{
		std::cout << "ERROR: Could not open /complete/log.txt file." << std::endl;
		return EXIT_FAILURE;
	}
	
	std::list<std::string>::iterator first_it;
	if( !last_line.empty()){
		//we know that the filename is the last 12 chars
		size_t pos = last_line.find_last_of("f");
		std::string first_frame = last_line.substr(pos, last_line.length() - pos);
		first_it = find(frames.begin(),frames.end(), first_frame);

		if(first_it != frames.end()){
			std::cout << "Last Tagging was exited on file: " << first_frame << std::endl;
			std::cout << "Tagging continuing from this frame." << std::endl;
		}
		else {
			std::cout << "ERROR: Last frame: " << first_frame << " could not be found in directory." << std::endl;
			return EXIT_FAILURE;
		}
	}
	else{
		std::cout<< "Starting from the first frame." << std::endl;
		first_it = frames.begin();
	}

	//now only run from first frame
	std::list<std::string>::iterator im_file;
	train_frame f;
	for(im_file = first_it; im_file != frames.end(); im_file++){
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
			//cv::imshow("WINDOW", inframe);
			//cv::waitKey(100);

			std::cout << "Opened: " << *im_file << std::endl;
			f = train_frame(*im_file, inframe, &s_boxes, &s_descriptors, &s_log);
			bool exit = false;
			if(tag_frames){
				f.tag_frame(&exit);
				if(exit){
					f.save_session(bag,s_tag, *first_it);
					return 0;
				}
			}
			if(tag_candidates){
				f.tag_candidates();
			}

		}
	}
	std::cout << "End of Frames." << std::endl;
	f.save_session(bag,s_tag, *first_it);

return 0;
}

/***************************************************************************

			Ugle copy of human_detection_2.cpp
			need to seperate this out into objects defined in one place
			i.e. what is below becomes a new object, and the ROS object is renamed

***************************************************************************/


std::vector<candidate> get_candidates(cv::Mat frame){

	float sigma = SIGMA;
	float Kdepth = KDEPTH;
	float Knormal = KNORMAL;
	int min_size = MIN_SIZE;
	int alpha = ALPHA, s = ESS;

	//params for conversion
	int in_width, in_height;
	int sub_width, sub_height;
	in_width = frame.size().width;
	in_height = frame.size().height;
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
				dval = frame.at<float>(y*alpha+dy,x*alpha+dx);
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

	//segment image
	universe *u_segmented = segment_image1C(inputIm, sigma, Kdepth, Knormal , min_size, &num_ccs, &normalIm, &depthseg, &normalseg, &outputIm);

	//merge regions
	std::vector<candidate> candidates = merge_and_filter(inputIm, u_segmented, sub_width, sub_height, frame);

	//Clean up image containers
	delete inputIm;
	delete outputIm;
	delete normalIm;
	delete depthseg;
	delete normalseg;
	delete u_segmented;
	
	return candidates;

}

void classify_candidates(std::vector<candidate> candidates, std::ofstream * descriptors, std::ofstream * log){

	int num_candidates = 0;
	int classification = 0;

	//Set up the callbacks
	cv::namedWindow("tag_candidate", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("tag_candidate",onMouse_cand, &classification);

	//show/evaluate candidates
	for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
		if( !(it->erased) ){
			classification = 0;
			descriptor candidate_descriptor = descriptor(it->im);
			candidate_descriptor.compute_descriptor();
			candidate_descriptor.check_human();
			cv::Mat display;
			double minval, maxval;
			cv::minMaxIdx(it->im, &minval, &maxval);
			it->im.convertTo(display, CV_8UC1,255.0/maxval);
			cv::imshow("tag_candidate", display);
			
			char c;
			int tmp;
			tmp = classification;
			//wait until classification is complete
			while(c != 32){
				c = (char)cv::waitKey(15);
				if( tmp != classification){
					tmp = classification;
					std::cout << "Classification: " << classification << std::endl;
				}
			}
			c = 'a';

			//
			candidate_descriptor.classification = classification;
			std::cout << "Classification: ";
			if(classification == 1){
				std::cout << "POSITIVE";
			}
			else{
				std::cout << "NEGATIVE";
			}
			std::cout << std::endl;

			//print candidates to file
			*descriptors << candidate_descriptor.HODstring();
			
			num_candidates++;
		}
	}
	cv::setMouseCallback("tag_Candidate",dummy);
	cv::destroyWindow("tag_candidate");
	*log << " candidates:" << num_candidates << std::endl;
	std::cout << std::endl << "END OF FRAME " << std::endl << std::endl;	
}

