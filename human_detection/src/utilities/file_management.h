#ifndef FILEMANAGEMENT_H
#define FILEMANAGEMENT_H


#include <dirent.h>
#include <string>
#include <time.h>
#include <sys/stat.h>
#include "../config.h"


/**********************COMMENTS***************************

READ AND WRITE CODE TAKEN FROM:
http://beansandbits.blogspot.co.uk/2012/07/readwrite-floating-point-images-with.html

************************************************************/


/************************************************************

				SAVE FLOAT FILES

************************************************************/
int writeMatToFile(const cv::Mat &I, std::string path) {
 
    //load the matrix size
    int matWidth = I.size().width, matHeight = I.size().height;
 
    //read type from Mat
    int type = I.type();
 
    //declare values to be written
    float fvalue;
    double dvalue;
    cv::Vec3f vfvalue;
    cv::Vec3d vdvalue;
 
    //create the file stream
    std::ofstream file(path.c_str(), std::ios::out | std::ios::binary );
    if (!file)
        return -1;
 
    //write type and size of the matrix first
    file.write((const char*) &type, sizeof(type));
    file.write((const char*) &matWidth, sizeof(matWidth));
    file.write((const char*) &matHeight, sizeof(matHeight));
 
    //write data depending on the image's type
    switch (type)
    {
    default:
        std::cout << "Error: wrong Mat type: must be CV_32F, CV_64F, CV_32FC3 or CV_64FC3" << std::endl;
        break;
    // FLOAT ONE CHANNEL
    case CV_32F:
        std::cout << "Writing CV_32F image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            fvalue = I.at<float>(i);
            file.write((const char*) &fvalue, sizeof(fvalue));
        }
        break;
    // DOUBLE ONE CHANNEL
    case CV_64F:
        std::cout << "Writing CV_64F image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            dvalue = I.at<double>(i);
            file.write((const char*) &dvalue, sizeof(dvalue));
        }
        break;
 
    // FLOAT THREE CHANNELS
    case CV_32FC3:
        std::cout << "Writing CV_32FC3 image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            vfvalue = I.at<cv::Vec3f>(i);
            file.write((const char*) &vfvalue, sizeof(vfvalue));
        }
        break;
 
    // DOUBLE THREE CHANNELS
    case CV_64FC3:
        std::cout << "Writing CV_64FC3 image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            vdvalue = I.at<cv::Vec3d>(i);
            file.write((const char*) &vdvalue, sizeof(vdvalue));
        }
        break;
 
    }
 
    //close file
    file.close();
 
    return 0;
}
 
int readFileToMat(cv::Mat &I, std::string path) {
 
    //declare image parameters
    int matWidth, matHeight, type;
 
    //declare values to be written
    float fvalue;
    double dvalue;
    cv::Vec3f vfvalue;
    cv::Vec3d vdvalue;
 
    //create the file stream
    std::ifstream file(path.c_str(), std::ios::in | std::ios::binary );
    if (!file)
        return -1;
 
    //read type and size of the matrix first
    file.read((char*) &type, sizeof(type));
    file.read((char*) &matWidth, sizeof(matWidth));
    file.read((char*) &matHeight, sizeof(matHeight));
 
    //change Mat type
    I = cv::Mat::zeros(matHeight, matWidth, type);
 
    //write data depending on the image's type
    switch (type)
    {
    default:
        std::cout << "Error: wrong Mat type: must be CV_32F, CV_64F, CV_32FC3 or CV_64FC3" << std::endl;
        break;
    // FLOAT ONE CHANNEL
    case CV_32F:
        std::cout << "Reading CV_32F image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &fvalue, sizeof(fvalue));
            I.at<float>(i) = fvalue;
        }
        break;
    // DOUBLE ONE CHANNEL
    case CV_64F:
        std::cout << "Reading CV_64F image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &dvalue, sizeof(dvalue));
            I.at<double>(i) = dvalue;
        }
        break;
 
    // FLOAT THREE CHANNELS
    case CV_32FC3:
        std::cout << "Reading CV_32FC3 image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &vfvalue, sizeof(vfvalue));
            I.at<cv::Vec3f>(i) = vfvalue;
        }
        break;
 
    // DOUBLE THREE CHANNELS
    case CV_64FC3:
        std::cout << "Reading CV_64FC3 image" << std::endl;
        for (int i=0; i < matWidth*matHeight; ++i) {
            file.read((char*) &vdvalue, sizeof(vdvalue));
            I.at<cv::Vec3d>(i) = vdvalue;
        }
        break;
 
    }
 
    //close file
    file.close();
 
    return 0;
}

/************************************************************

				DIRECTORY MANAGEMENT

************************************************************/

int check_make_dir(std::string path){

	/*sit = situation = -1 for fail
					  =  0 for existed
					  =  1 for created
	*/
	int sit = -1;
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (path.c_str())) != NULL) {
	  closedir (dir);
	  sit = 0;
	} 
	else {
		// could not open directory 
		//try to create one
		if(!mkdir(path.c_str(), 0777) ){
			sit = 1;
		}
	}
	return sit;
}

bool create_new_bag_tree(std::string bag){
		bool success = true;


		//create complete subtree and add blank files
		std::string path_complete = ANNOTDIR;
		path_complete.append(bag);
		path_complete.append("complete/");		
		if(mkdir(path_complete.c_str(), 0777)){
			std::cout << "create_new_bag_tree failed with bag " << bag << std::endl;
			success = false;
		}

		//opening these will create blank files
		FILE * pFile;
		pFile = fopen ( (path_complete+"log.txt").c_str(), "w" );
		if (pFile!=NULL){
			fclose (pFile);
		}

		pFile = fopen ( (path_complete+"boxes").c_str(), "w" );
		if (pFile!=NULL){
			fclose (pFile);
		}

		pFile = fopen ( (path_complete+"descriptors").c_str(), "w" );
		if (pFile!=NULL){
			fclose (pFile);
		}
		

		std::string path_session = ANNOTDIR;
		path_session.append(bag);
		path_session.append("session");		
		if(mkdir(path_session.c_str(), 0777)){
			std::cout << "create_new_bag_tree failed with bag: " << bag << std::endl;
			success = false;
		}

		return success;
}

bool create_new_session(std::string bag, std::string  * session){

	bool success = true;

	//get create a dir with the date
	time_t rawtime;
	struct tm * timeinfo;
	char dirname [20];
	time (&rawtime);

	timeinfo = localtime (&rawtime);

	strftime (dirname,20,"%Y_%m_%d-%H_%M",timeinfo);

	std::string path_session = ANNOTDIR;
	path_session.append(bag + "session/" + dirname + "/");

	if(mkdir(path_session.c_str(), 0777)){
		std::cout << "create_new_session failed with bag " << bag << std::endl;
		success = false;
	}

	//opening these will create blank files
	FILE * pFile;
	pFile = fopen ( (path_session+"log.txt").c_str(), "w" );
	if (pFile!=NULL){
		fclose (pFile);
	}

	pFile = fopen ( (path_session+"boxes").c_str(), "w" );
	if (pFile!=NULL){
		fclose (pFile);
	}

	pFile = fopen ( (path_session+"descriptors").c_str(), "w" );
	if (pFile!=NULL){
		fclose (pFile);
	}

	std::string s = dirname;
	*session = s;

	return success;

}

bool build_new_bag_tree(std::string bag){

	std::string root = DATADIR;
	std::string bagdir = root + bag;
	std::string tagsdir = bagdir + TAGS;
	std::string framesdir = bagdir + FRAMES;
	std::string descdir = bagdir + DESC;
	std::string compdir = bagdir + COMP;
	std::string t_log = bagdir + TAGLOG;
	std::string c_log = bagdir + CLASSIFLOG;

	std::list<std::string> dirs;
	dirs.push_back(bagdir);
	dirs.push_back(tagsdir);
	dirs.push_back(framesdir);
	dirs.push_back(compdir);
	dirs.push_back(descdir);
	dirs.push_back(descdir + METRIC + "/");
	dirs.push_back(descdir + SEG + "/");
	dirs.push_back(descdir + FILLED + "/");

	std::list<std::string> files;
	files.push_back(t_log);
	files.push_back(c_log);
	files.push_back(compdir + METRIC);
	files.push_back(compdir + SEG);
	files.push_back(compdir + FILLED);
	files.push_back(compdir + BOX_HITS);


	std::list<std::string>::iterator it;

	// 1)build bag directory

	// 2) build tags directory

	// 3) build descriptors directory

	// 4) build complete directory
	
	for(it = dirs.begin(); it != dirs.end(); it++){
		if(mkdir( it->c_str(), 0777)){
			std::cout << "ERROR: Build_new_bag_tree failed at " <<  *it << std::endl;
			return false;
		}
	}


	// 5) Create Tagging log file

	// 6) create classification log

	// 7) create complete files (4)

	FILE * pFile;

	for(it = files.begin(); it != files.end(); it++){
		pFile = fopen ( it->c_str(), "w" );
		if (pFile!=NULL){
			fclose (pFile);
		}
		else{
			std::cout << "ERROR: Build_new_bag_tree failed at " <<  *it << std::endl;
			return false;
		}
	}
	
	return true;
}

bool build_frame_files(std::string bag, std::string frame){

	std::string root = DATADIR;
	std::string bagdir = root + bag;
	std::string tagsdir = bagdir + TAGS + frame;
	std::string descdir = bagdir + DESC;

	std::list<std::string> files;
	files.push_back(tagsdir);
	files.push_back(descdir + METRIC + "/" + frame);
	files.push_back(descdir + SEG + "/" + frame);
	files.push_back(descdir + FILLED + "/" + frame);

	std::list<std::string>::iterator it;
	
	FILE * pFile;

	for(it = files.begin(); it != files.end(); it++){

		pFile = fopen ( it->c_str(), "w" );
		if (pFile!=NULL){
			fclose (pFile);
		}
		else{
			std::cout << "ERROR: build_frame_files failed at " <<  *it << std::endl;
			return false;
		}
	}
	return true;
}

std::list<std::string> get_frame_strs(std::string bag, bool &success){

	success = true;

	std::string path = DATADIR + bag + FRAMES;
	
	DIR *dir;
	struct dirent *ent;
	std::list<std::string> frames;
	if ((dir = opendir (path.c_str())) != NULL) {
		// print all the files and directories within directory 
		while ((ent = readdir (dir)) != NULL) {
			frames.push_back(ent->d_name);
		}
		closedir (dir);
	} 
	else {
		// could not open directory 
		perror ("");
		std::cout << "ERROR: " << path << " does not exist. " << std::endl;
		success = false;
	}

	//sort into numerical order
	frames.sort();
	frames.remove(".");
	frames.remove("..");

	//check that it is not empty
	if(frames.empty()){
		std::cout << "ERROR: " << path << " contains no frames. " <<std::endl;
		success = false;
	}

	return frames;
}



#endif
