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
	int complete_lines = 0;

	std::ofstream f_clog(c_log.c_str());

	//of and ifstreams to be opened within
	std::ofstream f_descfill;
	std::ofstream f_descmetric;
	std::ofstream f_descseg;

	std::list<std::string>::iterator im_file;
	//frame f;
	int num_candidates = 0;
	for(im_file = frames.begin(); im_file != frames.end(); im_file++){
		frame f;
		std::string path = DATADIR + bag + FRAMES + *im_file;
		int start_lines = complete_lines;
	

		std::list<box_tag>::iterator b_it;
		std::vector<candidate>::iterator c_it;

		//construct frame
		f = frame(path, load_success);
		if(!load_success){
			f_clog << "ERROR: Failed to load frame - " << *im_file << std::endl;
		}
		else{

			cv::Mat tmp = f.get_depthIm(false).clone();
			cv::imshow("window", tmp);

			std::string hack = "frame_00000.jsc68";

			//get list of boxes
			std::list<box_tag> tagged_boxes = get_tagged_boxes(bag, *im_file, load_success);
			if(!load_success){
				f_clog << "WARN: In frame - " << *im_file << ", failed to load some tags" << std::endl;
			}

			/*for (b_it = tagged_boxes.begin(); b_it != tagged_boxes.end(); b_it++ ){
				cv::rectangle(tmp, b_it->box, cv::Scalar(0,0,255), 2);
			}*/
			
			//get candidates
			f.get_candidates();

			if( f.valid_candidates() != 0) {
			
				//open individual frame files
				f_descfill.open( (descfill + *im_file).c_str() );
				f_descmetric.open( (descmetric + *im_file).c_str() );
				f_descseg.open( (descseg + *im_file).c_str() );
			
				//loop through each candidate
				for(c_it = f.candidates.begin(); c_it != f.candidates.end(); c_it++){
					//only for 'non-erased' candidates!
					if( !(c_it->erased) ){

						//show white candidate
						cv::rectangle(tmp, c_it->boundingBox, cv::Scalar(255,255,255), 2);

						//compare candidate with each tagged box to determine whether it is positive of not
						for (b_it = tagged_boxes.begin(); b_it != tagged_boxes.end(); b_it++ ){
							//draw blue box to show tag
							cv::rectangle(tmp, b_it->box, cv::Scalar(255,0,0), 2);
							cv::waitKey(WAIT);
							//determine whether positive of not
							if( b_it->is_cand_positive(c_it->boundingBox) ){
								b_it->inc_count();
								c_it->classification = 1;
								//show red for match
								cv::rectangle(tmp, c_it->boundingBox, cv::Scalar(0,0,255), 2);
								cv::waitKey(WAIT);
							}
						}

						cv::imshow("window", tmp);
						cv::waitKey(WAIT);
					
						//now get create and calc untouched descriptor
						descriptor d_metric = descriptor(c_it->im);
						d_metric.compute_descriptor();
						d_metric.classification = c_it->classification;

						/*TODO
						cv::Mat cand_filled = fill(it->im);				
						descriptor d_filled = descriptor(cand_filled);
						d_filled.compute_descriptor();
						d_filled.classification = c_it->classification;

						cv::Mat cand_segmented = get_segmented_cand(it->im);				
						descriptor d_seg = descriptor(cand_filled);
						d_seg.compute_descriptor();
						d_seg.classification = c_it->classification;
						*/				
				
						//now write to 'complete' files first
						f_compmetric << d_metric.HODstring();
						//f_compfill << d_filled.HODstring();
						//f_compseg << d_seg.HODstring();

						//count how many lines/descriptors have been written
						complete_lines++;

						//now write to individual frame folders
						f_descmetric << d_metric.HODstring();
						//f_descfill << d_filled.HODstring();
						//f_descseg << d_seg.HODstring();

						num_candidates++;

						//close opened files
						f_descmetric.close();
						//f_descfill.close();
						//f_descseg.close();
					}
				}
			}

			//and write to hits for each tagged box, from all candidates in frame
			int hits = 0;
			for (b_it = tagged_boxes.begin(); b_it != tagged_boxes.end(); b_it++ ){
				f_comphits << bag << " " << *im_file << " " << b_it->details() << std::endl;
				//count how many of the frames had hits
				if(b_it->get_count() > 0 ) hits++;
			} //We can evaluate tagged hit and miss rations using the counts values, and the vis value to determin what we count
			//record activity in classificatin log
			f_clog << bag.substr(0,bag.length()-1) << " " << *im_file << " " << "Boxes:" << tagged_boxes.size() << " hits:" << hits << " cands:" << num_candidates - start_lines << " lines:" << start_lines << "->" << num_candidates << std::endl;

				
		}
		f.erase_ims();
	}

	f_compfill.close();
	f_compmetric.close();
	f_compseg.close();
	f_comphits.close();
	f_clog.close();

	std::cout << "FINISHED" << std::endl;
	
	return 0;
}

