#include "config.h"
#include <stdlib.h>
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
#include "classification/box_tag.h"
#include "utilities/file_management.h"
#include "utilities/frame.h"


/*********************************************************************************
**************************  Current Comments  ************************************






*********************************************************************************
*********************************************************************************/

enum uiState_t { drawn, drawing, neutral };

class ui {

	public: 
	std::string im_filename;
	std::string bag;
	cv::Mat frame;

	std::ofstream * tags_file;
	std::ofstream * tag_log;

	ui();
	ui(std::string filename,std::string bag_, cv::Mat frame_, std::ofstream * tags_file_, std::ofstream * tag_log_);
	void tag_frame(bool &exit);

	cv::Point mouse;
	cv::Point box_start;
	cv::Point box_end;
	bool draw;
	uiState_t s;

	void draw_rectangle(cv::Point start, cv::Point end, const cv::Scalar& color, cv::Mat frame);	

};

void onMouse_frame( int event, int x, int y, int, void*);
void dummy( int event, int x, int y, int, void*);



/***********************************************************************

							ui methods


**********************************************************************/

ui::ui(){
}

ui::ui(std::string filename, std::string bag_, cv::Mat frame_, std::ofstream * tags_file_, std::ofstream * tag_log_){

	im_filename = filename;
	frame = frame_;
	tags_file = tags_file_;
	tag_log = tag_log_;
	bag = bag_;
}

//The beef!
void ui::tag_frame(bool &exit){

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
			*tag_log << im_filename << " boxes:" << count <<std::endl;
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
				std::cout << bag.substr(0,bag.length()-1) << " " << im_filename << " ";
				std::cout << box_start.x << " " << box_start.y << " ";
				std::cout << box_end.x - box_start.x << " " << box_end.y - box_start.y << " ";	
				std::cout << (int)vis << std::endl;

				*tags_file << bag.substr(0,bag.length()-1) << " " << im_filename << " ";
				*tags_file << box_start.x << " " << box_start.y << " ";
				*tags_file << box_end.x - box_start.x << " " << box_end.y - box_start.y << " ";	
				*tags_file << (int)vis << std::endl;
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
				exit = true;
				*tag_log << im_filename << " boxes:" << count << std::endl;
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


/***********************************************************************

							misc ui methods


**********************************************************************/

void onMouse_frame( int event, int x, int y, int flag, void* obj_){
	ui* obj = (ui*)obj_; 
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

void ui::draw_rectangle(cv::Point start, cv::Point end, const cv::Scalar& color, cv::Mat frame){
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
	bool success;

	//check that a data session is given
	if (bag_cstr == NULL){
		std::cout << "ERROR: No argument given, please pass the name of the bag file to be trained." << std::endl;
		return EXIT_FAILURE;
	}

	//append backslash
	std::string bag = bag_cstr;
	std::string::iterator last_char = bag.end();
	last_char--;

	if (*last_char != '/'){
		bag.append("/");
		//std::cout << "Added '/' to end of dir" << std::endl;
	}
	
	//get frames in directory
	std::list<std::string> frames = get_frame_strs(bag, success);
	if(!success){
		return EXIT_FAILURE;
	}

	//strings for opening files
	std::string root = DATADIR;
	std::string	bagdir = root + bag;
	std::string framesdir = bagdir + FRAMES;
	std::string tagsdir = bagdir + TAGS;
	std::string t_log = bagdir + TAGLOG;
	
	//Open Tag log file to find last entry
	std::ifstream tag_log_i(t_log.c_str() );
	//get to last line
	std::string line, last_line;
	if(tag_log_i.is_open() ){
		while ( std::getline (tag_log_i,line) ){
			last_line = line;
		}
		tag_log_i.close();
	}
	else{
		std::cout << "ERROR: Could not open Tagging_log file." << std::endl;
		return EXIT_FAILURE;
	}
	
	//now find iterator to first frame
	std::list<std::string>::iterator first_it;
	if( !last_line.empty()){
		//we know that the filename is the first 12 chars
		
		std::string first_frame = last_line.substr(0, 17);
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

	//open file for writing to
	std::ofstream tag_log( t_log.c_str(), std::ios::app );

	bool exit = false;

	//now go through everyfile
	std::list<std::string>::iterator im_file;
	for(im_file = first_it; im_file != frames.end(); im_file++){
		cv::Mat inframe;
		
		//read in and verify that data is valid
		readFileToMat(inframe, framesdir + *im_file);
		if (!inframe.data){
			std::cout << "Error : Path " << framesdir + *im_file << " cannot be loaded!!" << std::endl;
		}

		//open tags specific file
		std::ofstream tags_file( (tagsdir + *im_file).c_str() );	
		
		ui f = ui(*im_file, bag, inframe, &tags_file, &tag_log);
		f.tag_frame(exit);

		if(exit){
			return 0;
		}

	}

return 0;
}

