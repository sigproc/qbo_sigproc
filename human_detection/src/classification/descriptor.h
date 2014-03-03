#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>    // std::min
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <math.h>       /* isnan, sqrt */
#include <sstream>
#include <string>
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
		int classification;

		descriptor();
		~descriptor();
		descriptor(cv::Mat im);
		std::string HODstring();
		void compute_descriptor();
		bool check_human();
		cv::Mat visualise_cells(int s);
		cv::Mat visualise_gmags(int s);
		cv::Mat visualise_gdirs(int s);

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
	classification = -1;
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
	int k = 0;
	//iterate through cell start points
	for(int y = 0; y < im.rows; y+=HOD_CELL_SIZE){
		for(int x = 0; x < im.cols; x+= HOD_CELL_SIZE){
			
			cv::Mat histogram(18,1,CV_32F, cv::Scalar(0));
			//std::cout << "initialized histogram: " << histogram << std::endl;
			//*
			for(int i = 0; i < HOD_CELL_SIZE; i++){
				for(int j = 0; j < HOD_CELL_SIZE; j++){
					float dir = grad_dir.at<float>(cv::Point(x+i,y+j));
					float bin_exact = dir*HOD_ORIENT_BINS/360;
					int bin_floor = cvFloor(bin_exact);
					int bin_ceil = cvCeil(bin_exact);
					//int bin_floor = k%18;
					//int bin_ceil = k%18;
					float weight_floor = bin_ceil - bin_exact;
					float weight_ceil = 1 - weight_floor;
					bin_ceil = bin_ceil%18;
					float mag = grad_mag.at<float>(cv::Point(x+i,y+j));
					if(mag > 0){ //only add if mag > 0
						if(weight_floor*weight_ceil < 0){
						std::cout << "(" << x+i << "," << y+j << ") dir: " << dir << ", mag: " << mag << std::endl;
						std::cout << "Add to hist[" << bin_floor << "]: " << weight_floor*mag << std::endl;
						std::cout << "Add to hist[" << bin_ceil << "]: " << weight_ceil*mag << std::endl;					
						std::cout << "bin_floor: " << bin_floor << ", bin_ceil: " << bin_ceil << std::endl;		
						}			
						histogram.at<float>(cv::Point(0,bin_floor)) += weight_floor*mag;
						histogram.at<float>(cv::Point(0,bin_ceil)) += weight_ceil*mag;
					}
				}
			}
			if (!cells.empty()){
				//concat the cell's histogram to right
				//std::cout << "cells: " << cells << std::endl;
				//std::cout << "histogram: " << histogram << std::endl;
				cv::hconcat(cells, histogram,cells);
				//std::cout<< "cells, rows: " << cells.rows << ", cols: " << cells.cols <<std::endl;
			}
			else {
				cells = histogram;
			}
			//*/
			k++;
			
		}
	}
}

bool descriptor::check_human(){
	if(classification == 1) return true;
	else return false;
}

cv::Mat descriptor::visualise_cells(int s){
	//find maximum value in cells
	double minval, maxval;
    cv::minMaxIdx(cells, &minval, &maxval);
	//normalising factor so that the largest magnitude => a length of 8 pels
	float nf = 8.0/maxval;

	int cell_cols = im.cols/HOD_CELL_SIZE;
	int cell_rows = im.rows/HOD_CELL_SIZE;

	cv::Mat histograms(im.rows*s,im.cols*s,CV_8UC3, cv::Scalar(0,0,0));
	int j=0;
	//now for each cell, draw the histogram
	for(int y = 0; y < cell_rows; y++){
		for( int x = 0; x < cell_cols; x++){
			int cell_index = (y)*cell_cols+(x);
			cv::Mat cell_histogram = cells.col(cell_index);
			cv::Point hist_centre = cv::Point(HOD_CELL_SIZE*(x+0.5)*s, HOD_CELL_SIZE*(y+0.5)*s);
			for(int i = 0; i < cell_histogram.rows; i++){
				float angle = ((2*3.14159)*i)/HOD_ORIENT_BINS;
				float mag = cell_histogram.at<float>(i);
				//float mag = 32;
				//if(i == 11) mag = 32;
				if( mag < 0 ) std::cout << "Cell: " << cell_index << ", bin: " << i << ", mag: " << mag << std::endl;
				cv::Point arrow_end = hist_centre + cv::Point(nf*s*mag*cos(angle), nf*s*mag*sin(angle));
				line(histograms,hist_centre, arrow_end, cv::Scalar(0,0,255), 2, CV_AA);
			}
			//j++;
		}
	}
	std::cout << std::endl;
	return histograms;
}

cv::Mat descriptor::visualise_gdirs(int s){
	
	cv::Mat gradients(grad_mag.rows*s,grad_mag.cols*s, CV_8UC3, cv::Scalar(0,0,0));

	double minval, maxval;
    cv::minMaxIdx(grad_mag, &minval, &maxval);
	float scale = 10*s/maxval;
	

	for(int y = 0; y < gradients.rows; y+=2*s){
		for(int x = 0; x < gradients.cols; x+=2*s){
			float mag = grad_mag.at<float>(cv::Point(x/s,y/s));
			if( mag != 0){				
				float angle = 3.14159*grad_dir.at<float>(cv::Point(x/s,y/s))/180;
				//std::cout << "(" << x << "," << y << "), Mag: " << mag << ", Angle: " << angle << std::endl;
				cv::Point arrow_end = cv::Point(x+s,y+s) + cv::Point(mag*scale*cos(angle), mag*scale*sin(angle));
				line(gradients,cv::Point(x+s,y+s), arrow_end, cv::Scalar(0,0,255), 1, CV_AA);
			}
		}
	}
	return gradients;
}

cv::Mat descriptor::visualise_gmags(int s){
//increase size of grad_mag for visualisation
	cv::Mat gradients(grad_mag.rows*s,grad_mag.cols*s, CV_32FC1);
	for(int x = 0; x < gradients.cols; x++){
		for(int y = 0; y < gradients.rows; y++){		
			gradients.at<float>(cv::Point(x,y)) = grad_mag.at<float>(cv::Point(x/s,y/s));
		}
	}
	return gradients;
}

std::string descriptor::HODstring(){
	std::ostringstream stringStream;
	char sign = '-';
	if(classification == 1){
		sign = '+';
	}
	
	stringStream << sign << '1' << ' ';
	for( int j = 0; j < HOD.total(); j++){
		float f = HOD.at<float>(cv::Point(0,j));
		if( f != 0.0){
			stringStream << j << ':' << HOD.at<float>(cv::Point(0,j)) << ' ';
		}
	}
	stringStream << '#' <<'\n';
	return stringStream.str();
}

#endif
