#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>    // std::min
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <math.h>       /* isnan, sqrt */
#include "../segment_depth/segment.h"

#include "../config.h"

#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

class descriptor{
public:
		cv::Mat im;
		cv::Mat HOD;
		cv::Mat grad_dir;
		cv::Mat grad_mag;
		bool human;

		descriptor();
		~descriptor();
		descriptor(cv::Mat im);
		void compute_descriptor();
		void check_human();

private:

		cv::Mat cells;

		void calc_gradients();
		void compute_cells();

};

descriptor::descriptor(){	
}

descriptor::~descriptor(){
}

descriptor::descriptor(cv::Mat im_){
	im = im_;
	grad_dir = cv::Mat::zeros(im.size(), CV_32F);
	grad_mag = cv::Mat::zeros(im.size(), CV_32F);
	human = false;
}

void descriptor::calc_gradients(){

	float dx, dy;

	//std::cout << "input image of width: " << im.cols << ", height: " << im.rows << std::endl;

	//using kernel [-1 0 1] for dx and [-1 0 1]' for dy
	for(int x = 1; x < im.cols - 1; x++){
		for(int y = 1; y < im.rows - 1; y++){
			float sumx = 0, sumy = 0, deg = 0, mag = 0;
			//if pixel is defined set gradient orientation and mag, else leave is at zero
			if( !cvIsNaN(im.at<float>(cv::Point(x,y))) ){
				if( !cvIsNaN(im.at<float>(cv::Point(x-1,y))) && !cvIsNaN(im.at<float>(cv::Point(x+1,y))) ){
					sumx-= im.at<float>(cv::Point(x-1,y));
					sumx+= im.at<float>(cv::Point(x+1,y));
				}
				if( !cvIsNaN(im.at<float>(cv::Point(x,y-1))) && !cvIsNaN(im.at<float>(cv::Point(x,y+1))) ){
					sumy-= im.at<float>(cv::Point(x,y-1));
					sumy+= im.at<float>(cv::Point(x,y+1));
				}
				dx = sumx;
				dy = sumy;
				deg = cv::fastAtan2(dy,dx);
				if( (dx == 0) && (dy == 0)){
					mag = 0;
				}
				else{
					mag = sqrt(dx*dx+dy*dy);
				}
				//std::cout << "dx: " << dx << ", dy: " << dy << ", deg: " << deg << ", mag: " << mag <<std::endl;
			}
//			std::cout << "(" << x << "," << y << ") has dir " << deg << " degrees" << std::endl;
	//		std::cout << "(" << x << "," << y << ") has magnitude " << mag << std::endl;

			grad_dir.at<float>(cv::Point(x,y)) = deg;
			grad_mag.at<float>(cv::Point(x,y)) = mag;
		}
	}

}

void descriptor::compute_descriptor(){

	calc_gradients();
	compute_cells();

	//std::cout << "cells: " << cells << std::endl;

	//*/
	int cell_cols = im.cols/HOD_CELL_SIZE;
	int cell_rows = im.rows/HOD_CELL_SIZE;
	
	for(int x = 0; x < cell_cols - HOD_BLOCK_SHIFT; x+= HOD_BLOCK_SHIFT){
		for(int y = 0; y < cell_rows - HOD_BLOCK_SHIFT; y+= HOD_BLOCK_SHIFT){
			cv::Mat block_descriptor;
			cv::Mat cell_histogram;
			//now create vectors from blocks
			for (int j = 0; j < HOD_BLOCK_SIZE; j++){
				for (int i = 0; i < HOD_BLOCK_SIZE; i++){
					//get cell histogram of interest
					int cell_index = (y+j)*cell_cols+(x+i);
					//std::cout << "cell_index" << cell_index << std::endl;
					cell_histogram = cells.col(cell_index);
					//concat into block descriptor
					if(!block_descriptor.empty()){
						cv::vconcat(block_descriptor, cell_histogram, block_descriptor);
					}
					else{
						block_descriptor = cell_histogram;
					}
					
				}
			}
			//*/
			//normalize block descriptor
			normalize(block_descriptor, block_descriptor, 1, 0, cv::NORM_L2);
			//And add to image descriptor
			if(!HOD.empty()){
				cv::vconcat(HOD, block_descriptor, HOD);
			}
			else{
				HOD = block_descriptor;
			}
			//*/
		}
	}
	//*/
			
}

void descriptor::compute_cells(){
	
	//iterate through cell start points
	for(int x = 0; x < im.cols; x+= HOD_CELL_SIZE){
		for(int y = 0; y < im.rows; y+=HOD_CELL_SIZE){
			
			cv::Mat histogram(18,1,CV_32F, cv::Scalar(0));
			//std::cout << "initialized histogram: " << histogram << std::endl;

			//*
			for(int i = 0; i < HOD_CELL_SIZE; i++){
				for(int j = 0; j < HOD_CELL_SIZE; j++){
					float dir = grad_dir.at<float>(cv::Point(x+i,y+j));
					//std::cout << "Orientation at (" << x+i << "," << y+j << ") is: " << dir << std::endl;
					float bin_exact = dir/HOD_ORIENT_BINS;
					int bin_floor = cvFloor(bin_exact);
					int bin_ceil = cvCeil(bin_exact);
					float weight_floor = bin_ceil - bin_exact;
					float weight_ceil = 1 - weight_floor;
					//std::cout << "Add to hist[" << bin_floor << "]: " << weight_floor*grad_mag.at<float>(cv::Point(x+i,y+j)) << std::endl;
					//std::cout << "Add to hist[" << bin_ceil << "]: " << weight_ceil*grad_mag.at<float>(cv::Point(x+i,y+j)) << std::endl;
					//std::cout << "bin_floor: " << bin_floor << ", bin_ceil: " << bin_ceil << std::endl;					
					histogram.at<float>(cv::Point(0,bin_floor)) += weight_floor*grad_mag.at<float>(cv::Point(x+i,y+j));
					histogram.at<float>(cv::Point(0,bin_ceil)) += weight_ceil*grad_mag.at<float>(cv::Point(x+i,y+j));
					
				}
			}
			if (!cells.empty()){
				//concat the cell's histogram to right
				//std::cout << "cells: " << cells << std::endl;
				//std::cout << "histogram: " << histogram << std::endl;
				cv::hconcat(cells, histogram,cells);
			}
			else {
				cells = histogram;
			}
			//*/
			
		}
	}
}

void descriptor::check_human(){
	human = true;
}

#endif
