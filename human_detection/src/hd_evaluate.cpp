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

#define WAIT 100
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

	//create log name
	std::string c_log = bagdir + CLASSIFLOG;
	
	int complete_lines = 0;

	//std::ofstream f_clog(c_log.c_str());

	std::list<std::string>::iterator im_file;
	//frame f;


	//prepare variables for analysis
	int false_negatives_pre = 0;
	int false_negatives_post = 0;
	int false_positives = 0;
	int true_positives = 0;
	int true_negatives = 0;
	int tags = 0;

	//now analyse each frame
	for(im_file = frames.begin(); im_file != frames.end(); im_file++){
		frame f;
		std::string path = DATADIR + bag + FRAMES + *im_file;
		int start_lines = complete_lines;
	
		//initialise iterators
		std::list<box_tag>::iterator b_it;
		std::vector<candidate>::iterator c_it;

		//initialise per-frame counts
		int pre_class_tag_hits = 0;
		int post_class_tag_hits = 0;
		int human_detections = 0;
		int f_false_positives = 0;

		//construct frame
		f = frame(path, load_success);
		if(!load_success){
			//f_clog << "ERROR: Failed to load frame - " << *im_file << std::endl;
		}
		else{

			//get depth image to display
			cv::Mat tmp = f.get_depthIm(false).clone();
			cv::imshow("window", tmp);
			//cv::waitKey(10);

			//get list of boxes
			std::list<box_tag> tagged_boxes = get_tagged_boxes(bag, *im_file, load_success);
			if(!load_success){
				//f_clog << "WARN: In frame - " << *im_file << ", failed to load some tags" << std::endl;
			}

			//get candidates
			f.get_candidates();
			f.classify_candidates();
			tags = tagged_boxes.size();

			//now analyse tages
			if(tagged_boxes.size() == 0){
				//std::cout<<"No tags in frame" << std::endl;
				//If there are no humans in the image, proceed to work out if we have false positives or a true negative!
			
				//Have any humans been found in the scene?
				if(f.any_detections){
					//Detections have been made even though there are no humans in the image, so increase the number of false positives
					false_positives+=f.any_detections;
					f_false_positives+=f.any_detections;
				}
				else{
					//We have successfully detected no humans in the frame
					true_negatives++;
				}
			}
			//But if there are humans in the image, then have we detected them all?
			else{
				//std::cout<<"Tags in frame..." << std::endl;

				//determine if each tagged human in the scene has been identified as a valid candidate
				for (b_it = tagged_boxes.begin(); b_it != tagged_boxes.end(); b_it++ ){
					//draw blue box to show tag
					cv::rectangle(tmp, b_it->box, cv::Scalar(255,0,0), 2);
					//cv::waitKey(WAIT);

					//search candidates for matches to each tag
					for(c_it = f.candidates.begin(); c_it != f.candidates.end(); c_it++){
						//only for 'non-erased' candidates!
						if( !(c_it->erased) ){
							//determine whether positive or not
							if( b_it->is_cand_positive(c_it->boundingBox) ){
								b_it->inc_count(); //multiple valid regions might be of the same human in the frame. Take note.
								c_it->classification = 1;
								if(c_it->human){
									b_it->detected = true;
									//show red for match
									cv::rectangle(tmp, c_it->boundingBox, cv::Scalar(0,0,255), 2);
								}
								else{
									//show orange for failed at classification
									cv::rectangle(tmp, c_it->boundingBox, cv::Scalar(0,165,255), 2);
								}
								cv::imshow("window", tmp);
								//cv::waitKey(WAIT/2);
							}
						}
					}

					pre_class_tag_hits++;
					post_class_tag_hits++;
				
					//Having searched over all valid candidates lets anaylse result of each tag
					if(!b_it->get_count()){ // if there are no candidates matching the tag
						false_negatives_pre++;
						pre_class_tag_hits--;
						post_class_tag_hits--;
					}
					else if(!b_it->detected){
						false_negatives_post++;
						post_class_tag_hits--;
					}
					else{
						true_positives++;
						human_detections++;
					}
				}
				//std::cout<<"Boxes evaluated, now searching for false positives..." << std::endl;
				//finally search for false positives: i.e. candidates with classification = 1 but human = false
				for(c_it = f.candidates.begin(); c_it != f.candidates.end(); c_it++){
					if(!(c_it->erased)){
						if((c_it->human) && (c_it->classification == -1)){ //draw orange for a mis-classified candidate region
							false_positives++;
							f_false_positives++;
							cv::rectangle(tmp, c_it->boundingBox, cv::Scalar(204,0,139), 2);
						}
						else{
							if(c_it->classification != 1){ //draw any candidates not in region of any tagged boxes white
								cv::rectangle(tmp, c_it->boundingBox, cv::Scalar(255,255,255), 2);
							}
						}
					}
				}
				cv::imshow("window", tmp);
				//cv::waitKey(WAIT);

			}				
		}
		f.erase_ims();
		std::cout << std::endl << "Frame: " << *im_file << ", Tags: " << tags << std::endl;
		std::cout << "Candidates: " << f.candidates.size() << ", Human Detections: " << human_detections <<std::endl;
		std::cout << "Pre-Class hits: " << pre_class_tag_hits << ", Post-Class hits: " << post_class_tag_hits << ", False positives: " << f_false_positives << std::endl << std::endl << std::endl;

		float recall, precision;
		if(!(true_positives + false_negatives_post) == 0){
			recall = true_positives/(true_positives + false_negatives_post);
		}
		if(!(true_positives + false_positives)==0){
			precision = true_positives/(true_positives + false_positives);
		}

		std::cout << "Recall/Sensetivity: " << recall << ", Precision: " << precision << std::endl << std::endl;
		cv::waitKey(WAIT);
	}

	//f_clog.close();

	std::cout << "FINISHED" << std::endl;
	
	return 0;
}

