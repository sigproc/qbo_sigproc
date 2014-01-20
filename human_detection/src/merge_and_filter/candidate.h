#include <opencv2/core/core.hpp>
#include <algorithm>    // std::min
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "../segment_depth/segment.h"

#include "../config.h"

#ifndef CANDIDATE_H
#define CANDIDATE_H


class candidate {
	public:
		//descriptor requirements
		cv::Rect boundingBox;
		cv::Mat image;

		//TODO
		bool erased;

		//candidate features for quick heuristic rejection
		float max_inlier_fraction;
		float real_height;
		float real_width;
		cv::Point3f centre;

		//variables to help calculate candidate features
		int xmin, xmax, ymin, ymax;
		float depth_accumulator;

		//variables for other aids
		std::vector<cv::Point3f> pts;
		int id;

		//Constructors & destructors
		candidate();
		//candidate(int s);
		candidate(int x,int y, float z, int i);
		~candidate();

		//member functions
		void add(int x, int y, float z);
		bool merge(candidate c);
		void calc_centre();
		int size() const;
		void RANSAC_inliers();
		void calc_real_dims();
		float calc_real_distance(float depth, int p, int length, float fov);
		//void add_to_image(image<rgb> * im, rgb colour);
};

inline bool operator<(const candidate& lhs, const candidate& rhs){
	int lhsSize = lhs.size();
	int rhsSize = rhs.size();
	return lhsSize<rhsSize;
}

inline bool operator>(const candidate& lhs, const candidate& rhs){
	return operator<(rhs,lhs);
}

inline bool operator<=(const candidate& lhs, const candidate& rhs){
	return !operator>(lhs,rhs);
}

inline bool operator>=(const candidate& lhs, const candidate& rhs){
	return !operator<(lhs,rhs);
}

float candidate::calc_real_distance(float depth, int pel, int length, float fov){
	
	float fraction = (float)pel/(length - 1.0);
	return depth*( (fraction -0.5) )*tan(fov/2)*2; //I think there should be an extra factor of 2 in this term
}

candidate::candidate(int x,int y, float z, int i){
	xmin = x;
	ymin = y;
	xmax = x;
	ymax = y;
	depth_accumulator = z;
	erased = false; //TODO
	id = i;
	max_inlier_fraction = 0;
	//std::cout << "New Candidate: (" << x << "," << y << "," << z << ") with id: " << id << std::endl;
	pts.push_back(cv::Point3f(x,y,z));
}

candidate::candidate(){
}

candidate::~candidate(){

}

void candidate::add(int x, int y, float z){
	//TODO given that we are now keeping a record of the pts for each candidate, 
	//find bounding box variables	
	xmin = std::min(x,xmin);
	xmax = std::max(x,xmax);
	ymin = std::min(y,ymin);
	ymax = std::max(y,ymax);
	depth_accumulator += z;

	/*
	//calculate real position
	float rx = z*( (x/(PEL_WIDTH-1) -0.5) )*tan(F_H/2); //I think there should be an extra factor of 2 in this term
	float ry = z*( (y/(PEL_HEIGHT-1) -0.5) )*tan(F_V/2); //I think there should be an extra factor of 2 in this term
	float rz = z;
*/

	//std::cout << "Add: (" << x << "," << y << "," << z << ") to candidate with id: " << id << std::endl;
	pts.push_back(cv::Point3f(x,y,z));
}

int candidate::size() const{
	return pts.size();
}

void candidate::calc_real_dims(){
	float xleft = calc_real_distance(centre.z, xmin, PEL_WIDTH/ALPHA, F_H);
	float xright = calc_real_distance(centre.z, xmax, PEL_WIDTH/ALPHA, F_H);
	float ybottom = calc_real_distance(centre.z, (PEL_HEIGHT/(2*ALPHA) - ymax), PEL_HEIGHT/ALPHA, F_V);
	float ytop = calc_real_distance(centre.z, (PEL_HEIGHT/(2*ALPHA) - ymin), PEL_HEIGHT/ALPHA, F_V);

	real_height = ytop - ybottom;
	real_width = xright - xleft;
	/*std::cout << "Xmin: " << xmin << ", xmax: " << xmax << ", ymin: " << ymin << ", ymax: " << ymax <<std::endl;
	std::cout << "Depth: " << centre.z << " Xleft: " << xleft << ", xright: " << xright << ", ytop: " << ytop << ", ybottom: " << ybottom <<std::endl;
	std::cout << "Candidate: " << id << ", real_height: " << real_height << ", real_width: " << real_width << std::endl;*/
}
	

bool candidate::merge(candidate c){
	//note that c should be of a larger size than this
	if( c.size() < size() ){
		std::cout << "MERGE FAILED: CAN ONLY MERGE WITH A LARGER CANDIDATE" << std::endl;
		return false;
	}
	//find out new bounding box params	
	xmin = std::min(xmin, c.xmin);
	xmax = std::max(xmax, c.xmax);
	ymin = std::min(ymin, c.ymin);
	ymax = std::max(ymax, c.ymax);

	//use these to recalculate the real dimensions
	calc_real_dims();

	//add the points
	pts.insert(pts.end(), c.pts.begin(), c.pts.end());
	erased = true;
	return true;

}

/*void candidate::add_to_image(image<rgb> * im, rgb colour){
	
	for(std::vector<cv::Point3f>::iterator it = pts.begin(); it != pts.end(); it++){
      imRef(im, it->x, it->y) = colour;
	}

}*/
void candidate::RANSAC_inliers(){

	//find real points of each pel
	std::vector<cv::Point3f> realpts;
	std::vector<cv::Point3f>::iterator it1;
	for( it1 = pts.begin(); it1 != pts.end(); it1++){
 		float rx = calc_real_distance(it1->z, it1->x, PEL_WIDTH/ALPHA, F_H);
		float ry = calc_real_distance(it1->z, (PEL_HEIGHT/(2*ALPHA) - it1->y), PEL_HEIGHT/ALPHA, F_V);
		float rz = it1->z;
		//std::cout << "Realpt (" << rx << "," << ry << "," << rz << ")";
		//std::cout << " from pt: (" << it1->x << "," << it1->y << "," << it1->z << ")" <<std::endl;
		realpts.push_back(cv::Point3f(rx,ry,rz));
	}

	//initialise random seed
	srand (time(NULL));

	//loop through each iteration
	cv::Point3f p0;
	cv::Point3f p1;
	cv::Point3f p2;
	cv::Point3f p1p0;
	cv::Point3f p2p0;
	cv::Point3f pnormal;

	for(int k = 0; k < RANSACK; k++){		
		//randomly choose 3 pixels in candidate
		int inliers = 0;
		bool print = false;

		int p0_index = rand() % size();
		p0 = realpts.at(p0_index);
		
		int p1_index = rand() % size();
		while((p1_index == p0_index)){
			p1_index = rand() % size(); //ensure that p1_index isnt equal to p0_index
		}
		p1 = realpts.at(p1_index);

		int p2_index = rand() % size();
		bool tryagain = true;
		while(tryagain){
			tryagain = false;
			p2_index = rand() % size(); //ensure that p2_index isnt equal to p0_index or p1_index
			if(p2_index == p1_index) tryagain = true;
			if(p2_index == p0_index) tryagain = true;
		}
		p2 = realpts.at(p2_index);

		//compute vectors in plane
		p1p0 = p1-p0;
		p2p0 = p2-p0;

		//compute plane normal
		pnormal = p1p0.cross(p2p0);
		float L2norm = norm(pnormal);
		if( L2norm !=0 ){
			pnormal = pnormal * (1/L2norm);
			float offset = -p0.dot(pnormal);
			if(isnan(pnormal.x)){
				print = true;
			}

			//now count how many inliers
			for( std::vector<cv::Point3f>::iterator it = realpts.begin(); it != realpts.end(); it++){
				float d = std::abs(pnormal.dot(*it) + offset);
				//std::cout << "Point: (" << it->x << "," << it->y << "," << it->z << "), distance: " << d << std::endl;
				if(d < EPSILON){
					inliers++;
				}
			}
			///*
			if(print){
				std::cout << "P0 = (" << p0.x << "," << p0.y << "," << p0.z << ") "<< std::endl;
				std::cout << "P1 = (" << p1.x << "," << p1.y << "," << p1.z << ") "<< std::endl;
				std::cout << "P2 = (" << p2.x << "," << p2.y << "," << p2.z << ") "<< std::endl;
				std::cout << "Plane = (" << pnormal.x << "," << pnormal.y << "," << pnormal.z << "," << offset << ") "<< std::endl;
				std::cout << "NaN's found in Candidate: " << id << " of size: " << size() << std::endl;
			}
			/*
			std::cout << "Plane = (" << pnormal.x << "," << pnormal.y << "," << pnormal.z << "," << offset << ") "<< std::endl;
			std::cout << "Candidate " << id << " size:  " << size() << std::endl;
			std::cout << "Candidate " << id << " inliers:  " << inliers << std::endl;
			//*/
			//and update max_inliers
			max_inlier_fraction = std::max(float(inliers)/size(), max_inlier_fraction);
		}
	} 
	//std::cout << std::endl << "END OF CANDIDATE " << std::endl << std::endl << std::endl;
}

void candidate::calc_centre(){
	//TODO
	//find centre pixel from min and max pixel values
	int midx = 0.5*(xmin+xmax);
	int midy = 0.5*(ymin+ymax);
	float depth = depth_accumulator/size();

	//use true depth to convert real space location of candidate
	centre.x = calc_real_distance(depth, midx, PEL_WIDTH/ALPHA, F_H);
	centre.y = calc_real_distance(depth, midy, PEL_HEIGHT/ALPHA, F_V);
	centre.z = depth;

	/*//for now let us just use pel values
	centre[0] = pelx;
	centre[1] = pely;
	centre[2] = depth;*/

}

#endif
