#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
		cv::Mat im;

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
		void set_boundingBox();
		void create_candidate_image(cv::Mat &depthim);
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
	
	float pel_ratio = (float)pel/(length - 1.0);
	float angular_component = tan(fov/2);
	return depth*(pel_ratio - 0.5)*angular_component*2; //I think there should be an extra factor of 2 in this term

}

candidate::candidate(int x,int y, float z, int i){
	xmin = x;
	ymin = y;
	xmax = x;
	ymax = y;
	depth_accumulator = z;
	boundingBox = cv::Rect(xmin,ymin,xmax-xmin,ymax-ymin);
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

void candidate::set_boundingBox(){
	boundingBox = cv::Rect(xmin*ALPHA,ymin*ALPHA,(xmax-xmin+1)*ALPHA,(ymax-ymin+1)*ALPHA);
}

int candidate::size() const{
	return pts.size();
}

void candidate::calc_real_dims(){
	float xleft = calc_real_distance(centre.z, xmin, PEL_WIDTH/ALPHA, F_H*3.141592653589793/180);
	float xright = calc_real_distance(centre.z, xmax, PEL_WIDTH/ALPHA, F_H*3.141592653589793/180);
	float ybottom = calc_real_distance(centre.z, (PEL_HEIGHT/(2*ALPHA) - ymax), PEL_HEIGHT/ALPHA, F_V*3.141592653589793/180);
	float ytop = calc_real_distance(centre.z, (PEL_HEIGHT/(2*ALPHA) - ymin), PEL_HEIGHT/ALPHA, F_V*3.141592653589793/180);

	real_height = ytop - ybottom;
	real_width = xright - xleft;
	/*std::cout << "Xmin: " << xmin << ", xmax: " << xmax << ", ymin: " << ymin << ", ymax: " << ymax <<std::endl;
	std::cout << "Depth: " << centre.z << " Xleft: " << xleft << ", xright: " << xright << ", ytop: " << ytop << ", ybottom: " << ybottom <<std::endl;
	std::cout << "Candidate: " << id << ", real_height: " << real_height << ", real_width: " << real_width << std::endl;*/
}
	
void candidate::create_candidate_image(cv::Mat &depthim){
	//a value of -1 is invalid.

	int width = xmax-xmin+1;
	int height = ymax-ymin +1;

	cv::Mat oscm = cv::Mat::zeros(cv::Size(width,height),CV_8UC1); //original scale candidate mask
	std::cout << "Size of oscm: width = " << oscm.size().width << ", height = " << oscm.size().height << std::endl; 
	std::cout << "xmin = " << xmin <<", xmax = " << xmax << ", ymin = " << ymin <<", ymax = " << ymax << std::endl;

	//given that we only have a list of points, we first need to build the image representing these points
	for (std::vector<cv::Point3f>::iterator it = pts.begin(); it != pts.end(); it++){
		//std::cout << "x: " << it->x - xmin << ", y: " << it->y - ymin << std::endl;		
		oscm.at<uchar>(cv::Point(it->x-xmin,it->y-ymin)) = '1';
		//std::cout<< "Mask val: " << oscm.at<uchar>(cv::Point(it->x-xmin,it->y-ymin)) << std::endl;
	}

	//for a candidate_image of wc x hc, we must scale the image by min(wc/width*alpha, hc/height*alpha). Note we are still alpha too small!
	float scaling = std::min((float)CANDIDATE_WIDTH/(width*ALPHA), (float)CANDIDATE_HEIGHT/(height*ALPHA));
	//this gives us a new sub-candidate image within which we rebuild out scaled candidate
	cv::Mat nscm = cv::Mat(cv::Size(scaling*width, scaling*height), CV_8UC1); //New Scale Candidate Mask
	std::cout << "Scaling: " << scaling << ", new: width = " << nscm.size().width << ", height = " << nscm.size().height << std::endl; 
	//fill in this new scaled image
	for(int x = 0; x < nscm.size().width; x++){
		for(int y = 0; y<nscm.size().height; y++){
			int oldx = x/scaling;
			int oldy = y/scaling;
			//std::cout<< "new (" << x << "," << y << "), comes from old (" << oldx << "," << oldy << ")" << std::endl;
			nscm.at<uchar>(cv::Point(x,y)) = oscm.at<uchar>(cv::Point(oldx, oldy));
			//std::cout<< "Mask val at (" << x << "," << y << "): " << nscm.at<uchar>(cv::Point(x,y)) << std::endl;
		}
	}
	
	//now expand with square kernel, using the openCv dilation function
	cv::Mat element = getStructuringElement( cv::MORPH_RECT, cv::Size( DILATING_SCALE,DILATING_SCALE ));
	// Apply the dilation operation
	dilate( nscm, nscm, element);

	//im = nscm;

	//and fill in candidate region with pels from the original image
	cv::Mat fsci = cv::Mat::zeros(cv::Size(ALPHA*width*scaling, ALPHA*height*scaling), CV_32FC1); //final Scale Candidate image
//	cv::Mat ints = cv::Mat::zeros(cv::Size(ALPHA*width*scaling, ALPHA*height*scaling), CV_8UC1); //final Scale Candidate image
	
	//int lastcelly;

	for(int x = 0; x < fsci.size().width; x++){
		for(int y = 0; y < fsci.size().height; y++){
			int cellx = x/(ALPHA); //As the cell image has already been scaled, the only difference is alpha!
			int celly = y/(ALPHA);
			if(nscm.at<uchar>(cv::Point(cellx,celly)) == '1'){
				cv::Point origin_segmented = cv::Point(xmin, ymin)*ALPHA;
				cv::Point cell_segmented = origin_segmented + cv::Point(cellx*ALPHA, celly*ALPHA);
				cv::Point offset_segmented = cv::Point(x/scaling - cellx*ALPHA, y/scaling - celly*ALPHA);
				cv::Point depth_image_pel = cell_segmented+offset_segmented;
				/*/std::cout<<"Assigning (" << x << "," << y << ")";
				std::cout<<"from (" << depth_image_pel.x << "," << depth_image_pel.y << ") to: " << depthim.at<float>(depth_image_pel) << std::endl;
				std::cout<<"Candidate_image size: " << fsci.size().width << " by " <<fsci.size().height<< std::endl;
				std::cout<<"mask width: " << width << ", height: " <<height<< ", scaling: "<< scaling << std::endl;*/
				fsci.at<float>(cv::Point(x,y)) = depthim.at<float>(depth_image_pel);
			}
			else {
				fsci.at<float>(cv::Point(x,y)) = 0.0/0.0; //NaN i.e undefined

			}
			/*if(celly != lastcelly && celly > 0){
				cv::imshow("Building up of Candidate image", fsci);
				cv::waitKey(1);
				lastcelly = celly;
			}*/
		}
	}

	//finally we want to position this sub-candidate image into the candidate image
	im = cv::Mat(cv::Size(CANDIDATE_WIDTH, CANDIDATE_HEIGHT), CV_32FC1, 0.0/0.0);
	cv::Point image_centre = cv::Point((CANDIDATE_WIDTH)/2, (CANDIDATE_HEIGHT)/2);
	cv::Point start = image_centre - cv::Point(fsci.size().width/2, fsci.size().height/2);

	for(int x = 0; x < fsci.size().width; x++){
		for(int y = 0; y < fsci.size().height; y++){
			im.at<float>(start+cv::Point(x,y)) = fsci.at<float>(cv::Point(x,y));
		}
	}

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
 		float rx = calc_real_distance(it1->z, it1->x, PEL_WIDTH/ALPHA, F_H*3.141592653589793/180);
		float ry = calc_real_distance(it1->z, (PEL_HEIGHT/(2*ALPHA) - it1->y), PEL_HEIGHT/ALPHA, F_V*3.141592653589793/180);
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
	centre.x = calc_real_distance(depth, midx, PEL_WIDTH/ALPHA, F_H*3.141592653589793/180);
	centre.y = calc_real_distance(depth, midy, PEL_HEIGHT/ALPHA, F_V*3.141592653589793/180);
	centre.z = depth;

	/*//for now let us just use pel values
	centre[0] = pelx;
	centre[1] = pely;
	centre[2] = depth;*/

}

#endif
