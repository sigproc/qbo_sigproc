#include <opencv2/core/core.hpp>
#include "../segment_depth/segment.h"
#include "candidate.h"
#include <vector>
#include <algorithm>    // std::min
#include <stdio.h>
#include <iostream>
#include <map>

#include "../config.h"


#ifndef MERGENFILTER_H
#define MERGENFILTER_H

std::vector<candidate> merge_and_filter(image<float> *im, universe * u, int width, int height, cv::Mat &depthim){
	std::vector<candidate> candidates;
	std::map<int,candidate> components;

	//iterator for searching use
	std::map<int,candidate>::iterator it;

	//loop through each pixel in the image and add defined values
	for(int y=0; y < height; y++){
		for(int x = 0; x < width; x++){
			float z = imRef(im,x,y);
			if( abs(z) > VALID_VALUE){
							//get component id for pixel at (x,y)
				int id = u->find(y*width+x); //refer to the colouring algorithm in segment-image.h

				it = components.find(id);
				if(it == components.end()){
					//if id doesnt exist in map yet...
					//add new component with first pel at x,y, to the map
					candidate c = candidate(x,y,z,id);
					components.insert(std::map<int,candidate>::value_type(id, c));
				}
				else {
					//just add pel to candidate
					it->second.add(x,y,z);
				}
			}
		}
	}

	//now check that for the same id's u->size(x) == candidate.size.
	/*	
	for(int y=0; y < height; y+=10){
		for(int x = 0; x < width; x+=10){
			int id = u->find(y*width+x);
			std::map<int,candidate>::iterator it;
			it = components.find(id);
			std::cout << "Component: " << id << " has size: " << u->size(id) << std::endl;
			std::cout << "Candidate: " << id << " has size: " << it->second.size() << std::endl;
		}
	}
	//*/

	//for each candidate, calculate the centres of mass, and reject those candidates which are not sufficiently planar
	for(it = components.begin(); it != components.end(); it++){
		//calculate params for heuristic comparisons TODO we can remove the variables from the class have them just return here
		it->second.calc_centre(); 					// but its nice to have them in the class for now for reference
		it->second.RANSAC_inliers();
		it->second.calc_real_dims();
		float fraction = it->second.max_inlier_fraction;
		float height = it->second.real_height;
		float width = it->second.real_width;
		/*
		std::cout << "Size: " << it->second.size() << ", Depth: " << it->second.centre.z << ", w: " << width << ", h: " << height << ", fraction: " << fraction << std::endl;
		std::cout << "Height < MAX_HEIGHT: " << (height < CANDIDATE_MAX_HEIGHT) << std::endl;
		std::cout << "Width < MAX_WIDTH: " << (width < CANDIDATE_MAX_WIDTH) << std::endl;
		std::cout << "Fraction < MIN_INLIER_FRACTION: " << (fraction > MIN_INLIER_FRACTION) << std::endl;
		//*/
		
		//if components satisfy these, add them to candidates list
		if( ((fraction > MIN_INLIER_FRACTION) && (height < CANDIDATE_MAX_HEIGHT) && (width < CANDIDATE_MAX_WIDTH)) ){
			//std::cout << "Keep Candidate: " << it->second.id << " with fraction " << fraction << std::endl;
			candidates.push_back(it->second);
		} else {
			//std::cout << "Remove Candidate: " << it->second.id << " with fraction " << fraction << std::endl;
		}
		
	}
	
	//sort vector into order, largest first. (given we have overloaded the '<' operator to work with size)
	std::sort(candidates.rbegin(), candidates.rend());

	for(std::vector<candidate>::iterator itc = candidates.begin(); itc != candidates.end(); itc++){
		//std::cout << "Candidate Size: " << itc->size() << std::endl;
		int p_height = it->second.ymax - it->second.ymin;
		int p_width = it->second.xmax - it->second.xmin;
		int size = it->second.size();

		//if the candidate is smaller than the minimum width or height
		if( (itc->real_width < CANDIDATE_MIN_WIDTH) || (itc->real_height < CANDIDATE_MIN_HEIGHT) || (size/(p_height*p_width) < CANDIDATE_MIN_DENSITY) ){
			//search for larger candidates (i.e. from end() to where we are now; itc)
			cv::Point centrexz = cv::Point(itc->centre.x, itc->centre.z);
			///*			
			for(std::vector<candidate>::iterator itc1 = candidates.begin(); *itc1 > *itc ; itc1++){
				//std::cout << "Test Candidate Size: " << itc1->size() << std::endl;
				//merge if the following conditions are met
				cv::Point testxz = cv::Point(itc1->centre.x, itc1->centre.z);
				float xz_distance = cv::norm(centrexz.dot(testxz));
				float y_distance = abs(itc->centre.y-itc1->centre.y);
				
				if( (xz_distance < DELTAXZ) || (y_distance < DELTAY) ){

					if( itc->merge(*itc1) ){
						//if merging is successful, remove smaller candidate from list
						//itc1 = candidates.erase(itc1);
						//TODO
						//itc->erased = true;
					}
				}
			}//*/
		}
	}
	///*
	for(std::vector<candidate>::iterator itc = candidates.begin(); itc != candidates.end(); itc++){
		//std::cout << "Candidate Size: " << itc->size() << std::endl;
		if( (!itc-> erased) ){
			//std::cout << "Evaluating Candidate of Height: " << itc->real_height << ", Width: " << itc->real_width << std::endl;
			//std::cout << "Height < MIN_HEIGHT: " << (itc->real_height < CANDIDATE_MIN_HEIGHT) << std::endl;
			//std::cout << "Width < MIN_WIDTH: " << (itc->real_width < CANDIDATE_MIN_WIDTH) << std::endl;
			//now reject any candidates that are still minimum post merge
			if( (itc->real_width < CANDIDATE_MIN_WIDTH) || (itc->real_height < CANDIDATE_MIN_HEIGHT) ){
				itc->erased = true;
			}
		}
	}
	//*/

	//finally convert images into candidate images and calculate their bounding boxes
	for(std::vector<candidate>::iterator itc = candidates.begin(); itc != candidates.end(); itc++){
		if( (!itc-> erased) ){
			//initialise the bounding box
			itc->set_boundingBox();
			itc->create_candidate_image(depthim);
		}
	}
	return candidates;
}

#endif
