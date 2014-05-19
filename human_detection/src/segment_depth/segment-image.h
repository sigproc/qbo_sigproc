/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if #include <opencv2/core/core.hpp>not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#ifndef SEGMENT_IMAGE
#define SEGMENT_IMAGE

#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "filter.h"
#include "segment-graph.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#define NORMALWEIGHTSCALINGFACTOR 1//255/pi gives the values to be in a range that is expect for images (81)

//float angletotal = 0;
//int countangles = 0;

// random color
rgb random_rgb(){ 
  rgb c;
  double r;
  
  c.r = (uchar)random();
  c.g = (uchar)random();
  c.b = (uchar)random();
  return c;
}

rgb ordered_rgb(int i){
  rgb c;
  int imodmax = i%10648;
  int r = 0 +(10*imodmax)%200;
  int g = 2550 -(10*imodmax)%200;
  int b = 0 + (15*imodmax)%210;
  c.r = r;
  c.g = g;
  c.b = b;
  //std::cout << i << " " <<std::endl;
  return c;
}
  

/********************************************************************
********Function: diff								 ****************
********Description: returns euclidean distance		 ****************
******** 			 between two 3 channel pixel	 ****************
********			 values							 ****************
*********************************************************************/

// dissimilarity measure between pixels
static inline float diff(image<float> *r, image<float> *g, image<float> *b,
			 int x1, int y1, int x2, int y2) {
  return sqrt(square(imRef(r, x1, y1)-imRef(r, x2, y2)) +
	      square(imRef(g, x1, y1)-imRef(g, x2, y2)) +
	      square(imRef(b, x1, y1)-imRef(b, x2, y2)));
}

/********************************************************************
********Function: diff1C							 ****************
********Description: returns euclidean distance		 ****************
******** 			 between two single channel pixel****************
********			 values							 ****************
*********************************************************************/
// dissimilarity measure between pixels
static inline float diff1C(image<float> *d, int x1, int y1, int x2, int y2) {
  return sqrt(  square(  imRef(d, x1, y1)-imRef(d, x2, y2)  )   );
}

/********************************************************************
********Function: create_depth_graph				 ****************
********Description: Given depth image, returns graph****************
******** 			 object weighted by depth	     ****************
*********************************************************************/
edge* create_depth_graph(image<float> *d, int *edgeNum){
	
	int width = d->width();
	int height = d->height();

   // build graph
  edge *edges = new edge[width*height*4];
  int num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      if (x < width-1) {
	edges[num].a = y * width + x;
	edges[num].b = y * width + (x+1);
	edges[num].w = diff1C(d, x, y, x+1, y);
	num++;
      }

      if (y < height-1) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + x;
	edges[num].w = diff1C(d, x, y, x, y+1);

	num++;
      }

      if ((x < width-1) && (y < height-1)) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + (x+1);
	edges[num].w = diff1C(d, x, y, x+1, y+1);
	num++;
      }

      if ((x < width-1) && (y > 0)) {
	edges[num].a = y * width + x;
	edges[num].b = (y-1) * width + (x+1);
	edges[num].w = diff1C(d, x, y, x+1, y-1);
	num++;
      }

    }
  }
  *edgeNum = num;
  return edges;
}
/********************************************************************
********Function: anglediff					 		 ****************
********Description: Given normal image and 2 pixels ****************
******** 			 returns angular difference in rad***************
*********************************************************************/
float anglediff(image<cv::Vec3f> *normals, int x1, int y1, int x2, int y2) {
  
	float angleRAD = 0;
	//do the dot product of the two normal vectors
	float dota = (imRef(normals, x1, y1)[0])*(imRef(normals, x2, y2)[0]);
	float dotb = (imRef(normals, x1, y1)[1])*(imRef(normals, x2, y2)[1]);
	float dotc = (imRef(normals, x1, y1)[2])*(imRef(normals, x2, y2)[2]);
	float dotprod = dota+dotb+dotc;

	
	angleRAD = acos(dotprod); // acos returns values between 0 and pi (3.14)
	/*if ( !(x1 % 50) && !(y1 % 50) ){
		std::cout << "angleRAD: " << angleRAD << std::endl;
	}*/

	//std::cout << angleRAD << ", ";
	/*if(!isnan(angleRAD)){
		angletotal += angleRAD;
		countangles++;
		std::cout<< "Total: " <<angletotal<< ", Count:" << countangles << std::endl;
	}*/

	//TODO Work out a suitable, and logical scaling factor
	if(!isnan(angleRAD)){
		return angleRAD*NORMALWEIGHTSCALINGFACTOR;
	}
	else{
		return 0.0;
	}
}

/********************************************************************
********Function: create_normal_graph				 ****************
********Description: Given normal image, returns 	 ****************
******** 			 graph object				     ****************
*********************************************************************/

edge* create_normal_graph(image<cv::Vec3f> *normals, int *edgeNum){
	
	int width = normals->width();
	int height = normals->height();

   // build graph
  edge *edges = new edge[width*height*4];
  int num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
	
      if (x < width-1) {
	edges[num].a = y * width + x;
	edges[num].b = y * width + (x+1);
	edges[num].w = anglediff(normals, x, y, x+1, y);
	num++;
      }

      if (y < height-1) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + x;
	edges[num].w = anglediff(normals, x, y, x, y+1);

	num++;
      }

      if ((x < width-1) && (y < height-1)) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + (x+1);
	edges[num].w = anglediff(normals, x, y, x+1, y+1);
	num++;
      }

      if ((x < width-1) && (y > 0)) {
	edges[num].a = y * width + x;
	edges[num].b = (y-1) * width + (x+1);
	edges[num].w = anglediff(normals, x, y, x+1, y-1);
	num++;
      }

    }
  }
  *edgeNum = num;
  return edges;
}

/********************************************************************
********Function: post_process_components			 ****************
********Description: Merges connected components	 ****************
******** 			 which are smaller than min_size ****************
*********************************************************************/
void post_process_components(edge* edges, universe *u, int num, int min_size){
	// post process small components
  for (int i = 0; i < num; i++) {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
      u->join(a, b);
  }
}

/********************************************************************
********Function: create_imageFrom_universe			 ****************
********Description: Given a universe image, returns ****************
******** 			 rgb visualisation of connected  ****************
********			 components.					 ****************
*********************************************************************/

image<rgb>* create_imageFrom_universe(universe *u, int width, int height){

  //color output image for depth segmentation
  image<rgb> *output = new image<rgb>(width, height);
  // pick random colors for each component
  rgb *colors = new rgb[width*height];
  for (int i = 0; i < width*height; i++)
    colors[i] = random_rgb();
	//colors[i] = ordered_rgb(i);
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = u->find(y * width + x);
      imRef(output, x, y) = colors[comp];
    }
  }

  delete [] colors;  
  return output;
}
/********************************************************************
********Function: create_normal_image				 ****************
********Description: Given depth image and segmented ****************
******** 			 depth universe, creates normal Im **************
*********************************************************************/
image<cv::Vec3f>* create_normal_image(image<float>* d, universe *u){
	int width = d->width();
    int height = d->height();

	//create 'image' to hold normals
	image<cv::Vec3f>* normals = new image<cv::Vec3f>(width, height);
	//old normal tmp for handling unsolvable edge cases
	cv::Mat Oldabcd = cv::Mat::zeros(4,1,CV_32FC1);
	cv::Mat rowAboveabcd = cv::Mat::zeros(4,1,CV_32FC1);

	//for each pixel in the image
	for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {

		
		//get the component of the current pixel
		int comp = u->find(y * width + x);
		//initialise vector to hold points
		std::vector<cv::Vec4f> points;
		//add points if they share the same component as p(x,y)
		for (int i = -1; i <2; i++){
			//let j be y offset
			for( int j = -1; j <2; j++){
				//only for points with valid indices
				if( ((x+i) >= 0) && ((x+i) < width) && ((y+j) >= 0) && ((y+j) < height) ){
					//get comp from pixel relative to centre pel
					int chkcomp = u->find( (y+j)*width +(x+i) );
					if( chkcomp == comp){
						float intensity = imRef(d, (x+i), (y+j)) ;
						//std::cout << "Points considered for normal image: (" << x+i << "," << y+j << "," << intensity << ")" << std::endl;
						cv::Vec4f newpoint = cv::Vec4f((float)i,(float)j,intensity, 1.0);
						points.push_back(newpoint);
					}
				}
			}
		}
		
		//to print out every 10,000th result
		/*if ( !(x % 100) && !(y % 100) ){
				std::cout<<"Pel: (" <<  x << "," << y << "): " << std::endl;
				std::cout << " Points: " << std::endl;
				for (int i = 0; i < points.size(); i++ ){
					cv::Vec4f p = points.at(i);
					std::cout << "(";
					for (int j = 0; j <4; j++){
						std::cout << " " << p[j];
					}
					std::cout << " )" << std::endl;
					cv::waitKey(1);
				}
		}*/


		//Now we want to convert the the vector points, a list of 3xN vectors into a Mat
		cv::Mat pointsMat = cv::Mat(points). //convert vector in Mat,
							reshape(1);//make Nx4 1-channel Matrix out of Nx1 4-channel

		//create destination Mat and RHS zeros Mat
		cv::Mat abcd = cv::Mat::zeros(4,1,CV_32FC1);
		cv::Mat normal = cv::Mat::zeros(3,1, CV_32FC1);
		//solve
		if(pointsMat.rows >3){
			try{
				//cv::solve(pointsMat,RHS,normal,cv::DECOMP_SVD);
				cv::SVD::solveZ(pointsMat, abcd);
			}
			catch( cv:: Exception& e) {
				//if the above is not solvable, let normal equal the same as the previous
				abcd = cv::Mat::ones(4,1,CV_32FC1)*-1;
				
			}
			//only normalise validly found normals
			cv::normalize(abcd.rowRange(0,3), normal, 1, 0, cv::NORM_L2);	
		
		}
		else {
			//if we cant find the new normal, use the previously used one
			if( x == width-1){			
				//in such a case the next pel is on the next line at x = 0
				abcd = rowAboveabcd;
			}
			else{			
				abcd = Oldabcd;
			}
			//std::cout<<"No Normal for: (" <<  x << "," << y << ")" << std::endl;
			/*normal.at<float>(0) = 0.001;
			normal.at<float>(1) = 0.001;
			normal.at<float>(2) = 0.001;*/
		}
		//now normalise into normal
		cv::normalize(abcd.rowRange(0,3), normal, 1, 0, cv::NORM_L2);		
		Oldabcd = abcd;
		if(x == 0){
			//keep track of first pel in each row
			rowAboveabcd = abcd;
		}

		//now put normal into image<cv::Vec3f>
		imRef(normals,x,y)[0] = normal.at<float>(0);
		imRef(normals,x,y)[1] = normal.at<float>(1);
		imRef(normals,x,y)[2] = normal.at<float>(2);
		

		/*
		if ( !(x % 50) && !(y % 50) ){
			std::cout << "Normal: ( " << imRef(normals,x,y)[0] << " " << imRef(normals,x,y)[1] << " ";
			std::cout << imRef(normals,x,y)[2] << " )" << std::endl;
		}
		//*/
		}
	}

	//For debugging, rebuild normals from d
	/*for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
		imRef(normals,x,y)[0] = imRef(d, (x), (y)) ;
		imRef(normals,x,y)[1] = imRef(d, (x), (y)) ;
		imRef(normals,x,y)[2] = imRef(d, (x), (y)) ;
		}
	}*/

		

	return normals;


}

/********************************************************************
********Function: merge_segmentations				 ****************
********Description: Given two universes, returns	 ****************
******** 			 union of connected components   ****************
*********************************************************************/

universe* merge_segmentations(universe *u_depth, universe *u_normal, int width, int height){
    // loop through each pixel in new u_final, and assign component value
    //depending on which components the pixels belong to in u_depth and u_normal
	
	//ensure u_depth and u_normal are of the same size

	universe *u_final = new universe(width*height);
	std::map<int,int> components;
	
	//loop through each pixel, which will have a corresponding component in the new universe
	for (int y = 0; y < height; y++) {
    	for (int x = 0; x < width; x++) {
			int pelIndex = y*width+x;
			int d_comp = u_depth->find(pelIndex);
			int n_comp = u_normal->find(pelIndex);
			//compute a key unique to the combination of depth and normal components
			int mapKey = d_comp*width*height+n_comp;
			//see if this key has already been found.
			std::map<int,int>::iterator it = components.find(mapKey);
			if(it == components.end() ) {
				//this combination is new to the universe, so lets add its component to those present in u_final
				int f_comp = u_final->find(pelIndex);
				components.insert(std::pair<int,int>(mapKey, f_comp) );
			}
			else {
				//this combination has already been found, so join the new pixel to that component
				int compMatch = it->second;
				//now join the current pel in u_final, with the existing group
				u_final->join(pelIndex, compMatch);
			}
		}
	}

	return u_final;
}
/********************************************************************
********Function: visualise_normals					 ****************
********Description: Given normal image, returns rbg ****************
******** 			 visualisation of normal image   ****************
*********************************************************************/
image<rgb> *visualise_normals(image<cv::Vec3f> *normalsVec){
	
	int width = normalsVec->width();
    int height = normalsVec->height();	

	//first need to extend the range from 0->1 to 0->255
	image<rgb> *normalsRGB = new image<rgb>(width, height);
	//now scan through and apply multiplication
	for (int y = 0; y < height; y++) {
	    for (int x = 0; x < width; x++) {
			imRef(normalsRGB, x, y).r = (imRef(normalsVec,x,y)[0]+1)*128;
			imRef(normalsRGB, x, y).g = (imRef(normalsVec,x,y)[1]+1)*128;
			imRef(normalsRGB, x, y).b = (imRef(normalsVec,x,y)[2]+1)*128;
    	}
	}

	return normalsRGB;
}
/**************************************************************************
********Function: segment_image1C                          ****************
********Description: Given a single channel depth image,   ****************
********             returns RBG image representing any of:****************
********             Complete, Depth or Normal only 
********             segmentations.						   ****************
**************************************************************************/

universe *segment_image1C(image<float> * im, float sigma, float Kdepth, float Knormal, int min_size,
			  int * num_ccs, image<rgb> ** normalIm, image<rgb> ** depthseg, image<rgb> ** normalseg, image<rgb> ** output) {
  int width = im->width();
  int height = im->height();

  image<float> *d = new image<float>(width, height);
 
  //copy the image into *d before...  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(d, x, y) = imRef(im, x, y);
    }
  }
  // smoothing each color channel
  image<float> *smooth_d = smooth(d, sigma);
  delete d;

  //create graph from image, using distance
  int num_depth;
  edge* g_depth = create_depth_graph(smooth_d, &num_depth);

  // segment depth graph
  int depthThreshK = Kdepth;
  universe *u_depth = segment_graph(width*height, num_depth, g_depth, depthThreshK);

  // post process small components of depth graph
  post_process_components(g_depth, u_depth, num_depth, min_size);

  // create normal image
  image<cv::Vec3f> *normals = create_normal_image(smooth_d, u_depth);
  *normalIm = visualise_normals(normals);
  
  int num_normal;
  edge* g_normal = create_normal_graph(normals, &num_normal);

  //now both graphs are built delete images
  delete smooth_d;
  delete normals;

  //segment normal graph
  int normalThreshK = Knormal;
  universe *u_normal = segment_graph(width*height, num_normal, g_normal, normalThreshK);
  
  // post process small components of normal graph
  
	//TODO TODO TODO
  //post_process_components(g_normal, u_normal, num_normal, min_size);
	//TODO TODO TODO UNCOMMENT ^^^: Actually I think its better without!
  //finally, use both segmentations to merge segmentations into regions which are in the same components in
  // both u_normal and u_depth
  universe *u_final = merge_segmentations(u_depth, u_normal, width, height);

  //TODO properly
  post_process_components(g_depth, u_final, num_depth, min_size);

  delete [] g_depth;
  delete [] g_normal;

  //*num_ccs = u_depth->num_sets();
  *num_ccs = u_final->num_sets();
  //color output image for depth segmentation

  *depthseg = create_imageFrom_universe(u_depth, width, height);
  *normalseg = create_imageFrom_universe(u_normal, width, height);
  *output = create_imageFrom_universe(u_final, width, height);


  delete u_depth;
  delete u_normal;
  //delete u_final;

  return u_final;
}

#endif
