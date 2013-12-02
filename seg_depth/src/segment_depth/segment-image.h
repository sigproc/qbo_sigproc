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


/*//eigen includes
#define EIGEN2_SUPPORT
#include "../eigen/Eigen/LeastSquares"
//#include <Eigen/LeastSquares>*/




// random color
rgb random_rgb(){ 
  rgb c;
  double r;
  
  c.r = (uchar)random();
  c.g = (uchar)random();
  c.b = (uchar)random();
#include <opencv2/core/core.hpp>
  return c;
}

// dissimilarity measure between pixels
static inline float diff(image<float> *r, image<float> *g, image<float> *b,
			 int x1, int y1, int x2, int y2) {
  return sqrt(square(imRef(r, x1, y1)-imRef(r, x2, y2)) +
	      square(imRef(g, x1, y1)-imRef(g, x2, y2)) +
	      square(imRef(b, x1, y1)-imRef(b, x2, y2)));
}

/*
 * Segment an image
 *
 * Returns a color image representing the segmentation.
 *
 * im: image to segment.
 * sigma: to smooth the image.
 * c: constant for treshold function.
 * min_size: minimum component size (enforced by post-processing stage).
 * num_ccs: number of connected components in the segmentation.
 */
image<rgb> *segment_image(image<rgb> *im, float sigma, float c, int min_size,
			  int *num_ccs) {
  int width = im->width();
  int height = im->height();

  image<float> *r = new image<float>(width, height);
  image<float> *g = new image<float>(width, height);
  image<float> *b = new image<float>(width, height);

  // smooth each color channel  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(r, x, y) = imRef(im, x, y).r;
      imRef(g, x, y) = imRef(im, x, y).g;
      imRef(b, x, y) = imRef(im, x, y).b;
    }
  }
  image<float> *smooth_r = smooth(r, sigma);
  image<float> *smooth_g = smooth(g, sigma);
  image<float> *smooth_b = smooth(b, sigma);
  delete r;
  delete g;
  delete b;
 
  // build graph
  edge *edges = new edge[width*height*4];
  int num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (x < width-1) {
	edges[num].a = y * width + x;
	edges[num].b = y * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y);
	num++;
      }

      if (y < height-1) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + x;
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x, y+1);
	num++;
      }

      if ((x < width-1) && (y < height-1)) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y+1);
	num++;
      }

      if ((x < width-1) && (y > 0)) {
	edges[num].a = y * width + x;
	edges[num].b = (y-1) * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y-1);
	num++;
      }
    }
  }
  delete smooth_r;
  delete smooth_g;
  delete smooth_b;

  // segment
  universe *u = segment_graph(width*height, num, edges, c);
  
  // post process small components
  for (int i = 0; i < num; i++) {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
      u->join(a, b);
  }
  delete [] edges;
  *num_ccs = u->num_sets();

  image<rgb> *output = new image<rgb>(width, height);

  // pick random colors for each component
  rgb *colors = new rgb[width*height];
  for (int i = 0; i < width*height; i++)
    colors[i] = random_rgb();
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = u->find(y * width + x);
      imRef(output, x, y) = colors[comp];
    }
  }  

  delete [] colors;  
  delete u;

  return output;
}

// dissimilarity measure between pixels
static inline float diff1C(image<float> *d, int x1, int y1, int x2, int y2) {
  return sqrt(  square(  imRef(d, x1, y1)-imRef(d, x2, y2)  )   );
}

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

//TODO change type of input image
static inline float anglediff(image<cv::Vec3f> *normals, int x1, int y1, int x2, int y2) {
  
	float angleRAD = 0;
	//do the dot product of the two normal vectors
	float dota = (imRef(normals, x1, y1)[0])*(imRef(normals, x2, y2)[0]);
	float dotb = (imRef(normals, x1, y1)[1])*(imRef(normals, x2, y2)[1]);
	float dotc = (imRef(normals, x1, y1)[2])*(imRef(normals, x2, y2)[2]);
	float dotprod = dota+dotb+dotc;

	//TODO
	angleRAD = acos(dotprod);
	return angleRAD*180;
}

//TODO change type of input image
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





void post_process_components(edge* edges, universe *u, int num, int min_size){
	// post process small components
  for (int i = 0; i < num; i++) {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
      u->join(a, b);
  }
}

image<rgb>* create_imageFrom_universe(universe *u, int width, int height){

  //color output image for depth segmentation
  image<rgb> *output = new image<rgb>(width, height);
  // pick random colors for each component
  rgb *colors = new rgb[width*height];
  for (int i = 0; i < width*height; i++)
    colors[i] = random_rgb();
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = u->find(y * width + x);
      imRef(output, x, y) = colors[comp];
    }
  }

  delete [] colors;  
  return output;
}

image<cv::Vec3f>* create_normal_image(image<float>* d, universe *u){
	int width = d->width();
    int height = d->height();

	//create 'image' to hold normals
	image<cv::Vec3f>* normals = new image<cv::Vec3f>(width, height);
	//old normal tmp for handling unsolvable edge cases
	cv::Mat Oldnormal = cv::Mat::zeros(4,1,CV_32FC1);

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
		cv::Mat normal = cv::Mat::zeros(4,1,CV_32FC1);
		cv::Mat RHS = cv::Mat::zeros(pointsMat.rows,1, CV_32FC1);
		//solve
		if(pointsMat.rows >3){
			try{
				//cv::solve(pointsMat,RHS,normal,cv::DECOMP_SVD);
				cv::SVD::solveZ(pointsMat, normal);
			}
			catch( cv:: Exception& e) {
				//if the above is not solvable, let normal equal the same as the previous
				normal = cv::Mat::ones(4,1,CV_32FC1)*-1;
				
			}
		}
		else {
			//Oldnormal = normal;
			//std::cout<<"No Normal for: (" <<  x << "," << y << ")" << std::endl;
		}
		//now put normal into image<cv::Vec3f>
		imRef(normals,x,y)[0] = normal.at<float>(0);
		imRef(normals,x,y)[1] = normal.at<float>(1);
		imRef(normals,x,y)[2] = normal.at<float>(2);


		/*if ( !(x % 100) && !(y % 100) ){
			std::cout << "Normal: ( " << imRef(normals,x,y)[0] << " " << imRef(normals,x,y)[1] << " ";
			std::cout << imRef(normals,x,y)[2] << " )" << std::endl;
		}*/

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

universe* merge_segmentations(universe *u_depth, universe *u_normal){
	//TODO
    // loop through each pixel in new u_final, and assign component value
    //depending on which components the pixels belong to in u_depth and u_normal
	return u_depth;
}

image<rgb> *segment_image1C(image<float> *im, float sigma, float c, int min_size,
			  int *num_ccs) {
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
  int depthThreshK = c;
  universe *u_depth = segment_graph(width*height, num_depth, g_depth, depthThreshK);

  // post process small components of depth graph
  post_process_components(g_depth, u_depth, num_depth, min_size);

  //TODO create normal image
  image<cv::Vec3f> *normals = create_normal_image(smooth_d, u_depth);
  
  int num_normal;
  edge* g_normal = create_normal_graph(normals, &num_normal);

  //now both graphs are built delete images
  delete smooth_d;
  delete normals;

  //segment normal graph
  int normalThreshK = 300;
  universe *u_normal = segment_graph(width*height, num_normal, g_normal, normalThreshK);
  
  // post process small components of normal graph
  post_process_components(g_normal, u_normal, num_normal, min_size);

  delete [] g_depth;
  delete [] g_normal;

  //finally, use both segmentations to merge segmentations into regions which are in the same components in
  // both u_normal and u_depth
  //universe *u_final = merge_segmentations(u_depth, u_normal);

  //*num_ccs = u_depth->num_sets();
  *num_ccs = u_normal->num_sets();
  //color output image for depth segmentation
  //image<rgb> *output = create_imageFrom_universe(u_depth, width, height);
  image<rgb> *output = create_imageFrom_universe(u_normal, width, height);

  delete u_depth;
  delete u_normal;
  //delete u_final;

  return output;
}


/*image<float>* create_normal_image(image<float>* d, universe *u){
	int width = d->width();
    int height = d->height();

	//create 'image' to hold normals
	image<Eigen::Vector3d> normals = new image<Eigen::Vector3d>(width, height);

	//for each pixel in the image
	for (int y = 0; y < height; y++) {

      for (int x = 0; x < width; x++) {


		int numpoints = 0;
		//get the component of the current pixel

		int comp = u->find(y * width + x);
		//add points if they share the same component as p(x,y)
		//get number of points required
		for (int i = -1; i <2; i++){
			//let j be y offset

			for( int j = -1; j <2; j++){
				//get comp from pixel relative to centre pel
				int chkcomp = u->find( (y+j)*width +(x+i) );
				//if comp increase count
				if( chkcomp == comp){

					numpoints++;
				}
			}
		}



		// now we know how big points needs to be, we do it again, but this time allocate the points
		Eigen::Vector3d points[numpoints];
		numpoints = 0;
		//let i be x offset
		for (int i = -1; i <2; i++){

			//let j be y offset
			for( int j = -1; j <2; j++){
				//get comp from pixel relative to centre pel
				int chkcomp = u->find( (y+j)*width +(x+i) );
				//if comp is the same, add to points for regression

				if( chkcomp == comp){
					float intensity = imRef(d, (x+i), (y+j)) ;
					//points[numpoints] = Eigen::Vector3d ( (float)i, (float)j, intensity ); //TODO
					numpoints++;
				}

			}
		}
		//get coeffs to point to representative element in the 'image' normals
		Eigen::Vector3d coeffs = imRef(normals, x,y);
		//now use eigen to do regression

		//Eigen::linearRegression(numpoints, &points, &coeffs, 2);//TODO

		//coeffs now represent normal
		//TODO

		}

	}

	return d;


}*/


#endif
