#include "config.h"

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include "utilities/file_management.h"
#include <dirent.h>
#include <sys/stat.h>


/*********************************************************************************
**************************  Current Comments  ************************************




*********************************************************************************
*********************************************************************************/

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

 
/**********************************************8


***********************************************/

class save_frames
{
 
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber sub_;
	std::string bag;
	int frame_count;

  
public:
	save_frames()
	: it_(nh_)
	{
    // Subscrive to input video feed and publish output video feed
    sub_ =it_.subscribe("/camera/depth/image", 1, &save_frames::imageCallback, this);
	/*//directory = "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/bag_frames/";
	directory = DATADIR;*/

	//initialise frame count
	frame_count = 0;
	std::string dir;
	//try and grab "dir" value
	ros::NodeHandle p("~");
	p.getParam("dir", dir);
	ros::param::get("dir", dir);
	std::cout << "bag file is read as " << dir << std::endl;
	
	size_t pos = dir.find_first_of(".");
	bag = dir.substr(0, pos);

	//append backslash
	std::string::iterator last_char = bag.end();
	last_char--;

	if (*last_char != '/'){
		bag.append("/");
		//std::cout << "Added '/' to end of dir" << std::endl;
	}

	//directory = directory+dir+"/";
	//std::cout<< "Directory = $" << directory << std::endl;
	ROS_INFO("Bag file %s", bag.c_str());
	
	}

	/*save_frames(std::string bag_)
	: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		sub_ =it_.subscribe("/camera/depth/image", 1, &save_frames::imageCallback, this);
		//set bag variable and initialise count
		bag = bag_;
		frame_count = 0;
	}*/

	~save_frames(){
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& original_image);	

};

/***************************************************************

*********************************************************************/
//This function is called everytime a new image is published
void save_frames::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

    cv_bridge::CvImagePtr in_msg;

    try
    {
        in_msg = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("ROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 
	//image is stored as 32FC1 in in_msg->image which is of type Mat
	// container for frame name
	char frame_cstr[100];
	//container for 5 chars
	std::string five_chars;
	//pre-pend zeros so depending on size of number so that frame name is a constant length
	if ( (frame_count/10000) == 0)		five_chars.append("0");
	if ( (frame_count/1000) == 0)		five_chars.append("0");
	if( (frame_count/100) == 0)			five_chars.append("0");
	if( (frame_count/10) == 0) 			five_chars.append("0");
	//container for integer count
	char buffer[50];
	sprintf(buffer, "%d", frame_count);
	//increase frame count
	frame_count++;
	//append integer count to zeros for 5 chars
	five_chars.append(buffer);
	//put 5 chars into frame name
	sprintf(frame_cstr, "frame_%s.jsc68", five_chars.c_str());
	//convert to string
	std::string frame = frame_cstr;
	ROS_INFO("Frame string: %s", frame.c_str());
	//add directories
	std::string write_frame_to = DATADIR+bag+FRAMES;
	std::string tmp;
	//check bag/frames directory exists; if not, make it!
	DIR *dir;
	struct dirent *ent;
	dir = opendir(write_frame_to.c_str());
	if ( dir == NULL ) {// if bag/frames cant be opened...
		ROS_INFO("Dir: %s cant be opened", write_frame_to.c_str());
		//see if the bag file folder exists	
		closedir (dir);
		tmp = DATADIR + bag;
		dir = opendir( tmp.c_str());
		if( dir == NULL){ // if bag folder cant be opened
			closedir(dir);
			//build whole new tree!
			if(!build_new_bag_tree(bag)){
				return;
			}
		}
	}
	else{
		closedir (dir);
	}
	//std::cout << "bag: " << bag << " frame: " << frame << std::endl;
	//make directies for frames and write to the frame one
	if( build_frame_files(bag, frame) ){
			writeMatToFile(in_msg->image, (write_frame_to + frame).c_str() );
	}
	
	/*
	cv::Mat inframe;
	readFileToMat(inframe, filename);

	int in_width = in_msg->image.size().width;
	int in_height = in_msg->image.size().height;

	float cumsum = 0;

	for (int y=0; y < in_height; y++){
		for(int x=0; x < in_width; x++){
			float loaded = inframe.at<float>(y,x);
			float msg = in_msg->image.at<float>(y,x);

			//std::cout << "At (" << x << "," << y << "): " << std::endl;
			//std::cout << "Msg image has val: " << msg;
			//std::cout << " and Loaded image has val: " << loaded << std::endl;
			if( !isnan(loaded) && !isnan(msg)){
				//std::cout << "Difference: " << msg-loaded << std::endl;
				cumsum+= msg-loaded;
			}
		}
	}

	std::cout << "difference between loaded image and message is " << cumsum <<  std::endl;
	std::cout << "Displaying image" << std::endl;

	cv::imshow("WINDOW", inframe);
	cv::waitKey(1000);*/
	
}




/********************************************************

*******************************************************/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_frames");

	//Get the argument: the name of the bag file.
	//const char* bag_cstr = argv[1];

	//check that an argument is given
	//if (bag_cstr == NULL){
	//	std::cout << "ERROR: No argument given, please pass the name of the bag file to be conveted." << std::endl;
	//	return EXIT_FAILURE;
	//}

	//convert into string and add backslash if it does not exist
	/*std::string bag = bag_cstr;
	std::string::iterator last_char = bag.end();
	last_char--;

	if (*last_char != '/'){
		bag.append("/");
		//std::cout << "Added '/' to end of dir" << std::endl;
	}

	//call constructor with argument*/
	//save_frames hd(bag);
	save_frames hd;
	ros::spin();
	return 0;
}

