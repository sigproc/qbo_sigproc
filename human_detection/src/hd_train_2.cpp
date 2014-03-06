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

	//create desc roots (to be append by frame)
	std::string descfill = descdir + FILLED;
	std::string descmetric = descdir + METRIC;
	std::string descseg = descdir + SEG;
	//create comp filenames
	std::string compfill = compdir + FILLED;
	std::string compmetric = compdir + METRIC;
	std::string compseg = compdir + SEG;
	std::string comphits = compdir + BOX_HITS;
	//create log name
	std::string c_log = bagdir + CLASSIFLOG;
	


	//open comp and log files (For the output files, these will be created - and are open for the entire run)
	std::ofstream f_compfill(compfill.c_str());
	std::ofstream f_compmetric(compmetric.c_str());
	std::ofstream f_compseg(compseg.c_str());
	std::ofstream f_comphits(comphits.c_str());

	std::ofstream f_clog(c_log.c_str());

	//of and ifstreams to be opened within
	std::ofstream f_descfill;
	std::ofstream f_descmetric;
	std::ofstream f_descseg;

	std::list<std::string>::iterator im_file;
	frame f;
	for(im_file = frames.begin(); im_file != frames.end(); im_file++){
		std::string path = DATADIR + bag + FRAMES + *im_file;
		//construct frame
		f = frame(path, load_success);
		if(!load_success){
			f_clog << *im_file << " :failed to load frame" << std::endl;
		}
		else{

			cv::Mat tmp = f.get_depthIm(false).clone();
			cv::imshow("window", tmp);

			std::string hack = "frame_00000.jsc68";

			//get list of boxes
			std::list<box_tag> tagged_boxes = get_tagged_boxes(bag, /**im_file*/hack, load_success);
			if(!load_success){
				f_clog << *im_file << " :failed to load some tags" << std::endl;
			}

			std::list<box_tag>::iterator it;
			for (it = tagged_boxes.begin(); it != tagged_boxes.end(); it++ ){
				cv::rectangle(tmp, it->box, cv::Scalar(0,0,255), 2);
			}
			
			
			cv::imshow("window", tmp);

		}
		cv::waitKey(100);
		
	}
	
return 0;
}

