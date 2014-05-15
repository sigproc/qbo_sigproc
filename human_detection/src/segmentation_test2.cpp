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

#include "config.h"

#define WAIT 50
/*********************************************************************************
**************************  Current Comments  ************************************






*********************************************************************************
*********************************************************************************/
 
/***************************************************************************************

										methods

***************************************************************************************/

std::list<box_tag> get_tagged_boxes(std::string bag, std::string filename, bool &success){

	success = true;

	std::list<box_tag> output;

	std::string root = DATADIR;
	std::string	bagdir = root + bag;
	std::string tagsdir = bagdir + TAGS;
	
	std::ifstream tags_file( (tagsdir + filename).c_str() );

	std::string line;

	if( tags_file.is_open() ){
		while ( std::getline (tags_file,line) ){
			//fill list with boxes, each parsed from one line of the file
			//std::cout << line << std::endl;
			box_tag bt = box_tag(line);
			//std::cout << "frame_check " << bt.verify_frame(filename) << std::endl;
			//std::cout << "bag_check " << bt.verify_bag(bag) << std::endl;
			
			if (bt.verify_frame(filename) && bt.verify_bag(bag)){
				output.push_back( box_tag(line) );
			}
			else{
				success = false;
			}
		}
		tags_file.close();
	}
	else{
		std::cout << "ERROR: tags file, " << tagsdir + filename << "could not be opened." << std::endl;
		success = false;
	}

	return output;

}

/***************************************************************************************

										Main

***************************************************************************************/
int main ( int argc, char **argv){

	//The argument is the name of the bag file.
	const char* bag_cstr = argv[1];
	bool load_success;

	//check that a data session is given
	if (bag_cstr == NULL){
		std::cout << "ERROR: No argument given, please pass the name of the bag file to tested." << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Segmentation Test Started" << std::endl;

	//append backslash
	std::string bag = bag_cstr;
	std::string::iterator last_char = bag.end();
	last_char--;

	if (*last_char != '/'){
		bag.append("/");
		//std::cout << "Added '/' to end of dir" << std::endl;
	}
	
	//get frames in directory
	std::list<std::string> frames = get_frame_strs(bag, load_success);
	if(!load_success){
		return EXIT_FAILURE;
	}

	//open all the files for reading and writing!!
	//create string roots
	std::string root = DATADIR;
	std::string	bagdir = root + bag;
	std::string compdir = bagdir + COMP;
	std::string descdir = bagdir + DESC;

	std::stringstream log_name;
	log_name << root << SEGTEST << "." << bag.substr(0,bag.length()-1)<<"."<<ALPHA<<"."<<ESS<<"."<<KDEPTH<<"."<<KNORMAL<<"."<<MIN_SIZE;
	std::string results_log = log_name.str()+".csv";
	//results_log = results_log + "." + bag.substr(0,bag.length()-1) + ".csv";

	int complete_lines = 0;
	int count = 0;

	std::ofstream f_resultslog(results_log.c_str());
	std::cout << "Log Directory: " << results_log << std::endl;
	std::cout << "Frames to be processed: " << frames.size() << std::endl;

	std::list<std::string>::iterator im_file;
	//frame f;
	int num_candidates = 0;
	for(im_file = frames.begin(); im_file != frames.end(); im_file++){
	//for(im_file = frames.end(); im_file != frames.begin(); im_file--){
		frame f;
		std::string path = DATADIR + bag + FRAMES + *im_file;
		int start_lines = complete_lines;
	

		//std::list<box_tag>::iterator b_it;
		std::vector<candidate>::iterator c_it;

		//construct frame
		f = frame(path, load_success);
		if(!load_success){
			f_resultslog << "ERROR: Failed to load frame - " << *im_file << std::endl;
		}
		else{

			//get candidates to initiate running processing
			f.get_candidates();
			cv::Mat tmp = f.get_depthseg(false).clone();
			//cv::imshow("window", tmp);
			//cv::waitKey(10);

			clock_t postPrep = f.postPrep;
			clock_t postSeg = f.postSeg;

			int Seg_ms = (postSeg - postPrep) * 1000 / CLOCKS_PER_SEC;

			//std::cout << "Time taken to segment each frame: " << std::endl;
			//std::cout << "Seg Processing: " << Seg_ms/1000 << "s " << Seg_ms%1000 << "ms " << std::endl;
			
			f_resultslog << Seg_ms <<std::endl;
			count++;

			//This is only for capturing the same frame from each set for different params
			
			if(bag == "raws/"){
				if(*im_file == SEGIMRAW){
					cv::imwrite((log_name.str()+".png").c_str(), tmp);
				}
			}
			if(bag == "bedroom1_f/"){
					if(*im_file == SEGIMBED){
						cv::imwrite((log_name.str()+".png").c_str(), tmp);
					}
			}
			if(bag == "corridor1_f/"){
					if(*im_file == SEGIMCOR){
						cv::imwrite((log_name.str()+".png").c_str(), tmp);
					}
			}
			if(bag == "kitchen1_f/"){
					if(*im_file == SEGIMKIT){
						cv::imwrite((log_name.str()+".png").c_str(), tmp);
					}
			}
			if(bag == "livingroom1_f/"){
					if(*im_file == SEGIMLIV){
						cv::imwrite((log_name.str()+".png").c_str(), tmp);
					}
			}
		}

		f.erase_ims();
		std::cout<< *im_file << std::endl;
		//if(!(count % 25))std::cout<<std::endl;
	}

	f_resultslog.close();

	std::cout << "FINISHED" << std::endl;
	
	return 0;
}

