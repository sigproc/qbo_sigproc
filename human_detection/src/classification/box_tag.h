#ifndef BOX_TAG_H

#define BOX_TAG_H


#include <stdio.h>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>

/*********************************************************************************
**************************  Current Comments  ************************************


*********************************************************************************
*********************************************************************************/

class box_tag{

	public:

	cv::Rect box;

	bool verify_frame(std::string str);
	bool verify_bag(std::string str);
	bool is_cand_positive(cv::Rect cand);
	std::string tagged_string();
	//std::string tag();
	void inc_count();
	
	void set_box(int tlx, int tly, int width, int height);
	void set_vis(int v);
	void set_bag(std::string bag);
	void set_frame(std::string frame);
	
	box_tag();
	box_tag(std::string tag_line); //create box object from tagged data file
	box_tag(int tlx_, int tly_, int width_, int height_, std::string bag_, std::string frame_, int vis_); //create object from human input


	private:

	int vis;
	std::string bag;
	std::string frame;
	int count;

};

box_tag::box_tag(std::string tag_line){ //for use in hd_train

	//parse line to get the variables required below

	/*//Position of string "frame" helps us find the bag name
	size_t f_pos = last_line.find_first_of("frame");
	//thus the bag name is the first f_pos-1 characters (removing the space) of the line
	bag = tag_line.substr(0, f_pos - 1);
	//Frame is always 17 characters
	frame = tag_line.substr(f_pos,17);*/
	int tlx, tly, width, height;
	char bag_buf[20], frame_buf[20];
	std::sscanf(tag_line.c_str(), "%s %s %d %d %d %d %d", bag_buf, frame_buf, &tlx, &tly, &width, &height, &vis);
	bag = bag_buf;
	frame = frame_buf;
	box = cv::Rect(tlx, tly, width, height);
	count = 0;
	//std::cout << "Read in: Bag: " << bag << " frame: " << frame << " tlx: " << tlx << " tly: " << tly << " width: " << width << " height: " << height << " vis: " << vis << std::endl;
}

box_tag::box_tag(int tlx_, int tly_, int width_, int height_, std::string bag_, std::string frame_, int vis_){ //for use in hd_tag
	set_box(tlx_, tly_, width_, height_);
	bag= bag_;
	frame =  frame_;
	vis = vis_;
	count = 0;
}

box_tag::box_tag(){
}

void box_tag::set_box(int tlx, int tly, int width, int height){
	box = cv::Rect(tlx,tly,width,height);
}

void box_tag::set_vis(int v){
	vis = v;
}

void box_tag::set_bag(std::string bag_){
	bag = bag_;
}
void box_tag::set_frame(std::string frame_){
	frame = frame_;
}

bool box_tag::verify_frame(std::string str){
	return str == frame;
}

bool box_tag::verify_bag(std::string str){
	//std::cout << "bag from file: " << bag << std::endl;
	//std::cout << "bag in analysis: " << str.substr(0,bag.length()) << std::endl;
	return bag == str.substr(0,bag.length());
}

bool box_tag::is_cand_positive(cv::Rect cand){

	int dx = 0, dy = 0;
	int clx = cand.x;
	int cty = cand.y;
	int crx = clx + cand.width;
	int cby = cty + cand.height;

	int tlx = box.x;
	int tty = box.y;
	int trx = tlx + box.width;
	int tby = tty + box.height;

	//get dx

	//Top line represents box(t's) verticle edges		:	  |	  |
	//														 tlx  trx
	// Bottom line represents cand(c's) verticle edges	:	|   |
	//													  clx  crx
	if (clx < tlx){
		if (crx < tlx){			//		| |
			dx = 0;				//	| |
		}
		else if (crx < trx){	//	  |   |
			dx = crx - tlx;		//	|   |
		}
		else{					//	  | |
			dx = trx - tlx;		//	| 	  |
		}
	}
	else if ( clx < trx){
		if(crx < trx){			//	|     |
			dx = crx - clx;		//	  | |
		}
		else {					//	|   |
			dx = trx - clx;		//	  |   |
		}
	}
	else {						//	| |
		dx = 0;					//		| |
	}

	//get dy

	//left line represents cand(c's) verticle edges		:	cty_
	//																tty_
	// right line represents box(t's) verticle edges	:	cby_	
	//													  			tby_
	if (cty < tty){				//	_
		if (cby < tty){			//	_	
			dy = 0;				//		_
		}						//		_
		//
		else if (cby < tby){	//	_
			dy = cby - tty;		//		_
		}						//  _
								//		_
		//
		else{					//	_
			dy = tby - tty;		//		_
		}						//		_
	}							//	_
	//
	else if ( cty < tby){		//		_
		if(cby < tby){			//	_
			dy = cby - cty;		//	_
		}						//		_
		//
		else {					//		_
			dy = tby - cty;		//	_
		}						//		_
	}							//	_
		//
	else {						//		_
		dy = 0;					//		_
	}							//	_
								//	_

	int overlap_area = dx * dy;
	int cand_area = cand.width * cand.height;

	if( cand_area != 0){
		if(overlap_area / cand_area > CAND_OVRLP_TRUE){
			return true;
		}
	}

	return false;

}

void box_tag::inc_count(){
	count++;
}

std::string box_tag::tagged_string(){
	char buffer[75];
	//std::string line = sprintf(buffer, "%s %s %d %d %d %d %d %d", bag.c_str(), frame.c_str(), box.x, box.y, box.width, box.height, vis, count);
	std::string line = "meh";
	return line;
}

#endif
