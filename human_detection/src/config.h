#ifndef CONFIG
#define CONFIG

//Select which images to show from running
#define SHOWDEPTHIM true
#define SHOWDEPTHSEG false
#define SHOWNORMALSEG false
#define SHOWNORMALIM false
#define SHOWJOINTSEG false
#define SHOWCANDIDATES true
#define SHOWBOXES true


//define system parameters
#define SIGMA 0.5
#define ALPHA 8
#define ESS 13
#define KDEPTH 5
#define KNORMAL 1
#define MIN_SIZE 15
#define CANDIDATE_WIDTH 64
#define CANDIDATE_HEIGHT 128
#define DILATING_SCALE 5

//define system parameters
/*#define SIGMA 0.5
#define ALPHA 8
#define ESS 13
#define KDEPTH 100 //This is very good. Keep! (with (alpha,8),(sigma,0.5), (ess,13), (min_size,20)
#define KNORMAL 10
#define MIN_SIZE 50*/

//merge and filter params
#define MIN_INLIER_FRACTION 0.1
#define VALID_VALUE 0.000001
#define CANDIDATE_MIN_WIDTH 0.2 //meters
#define CANDIDATE_MAX_WIDTH 1.2 //meters
#define CANDIDATE_MIN_HEIGHT 1.0 //meters
#define CANDIDATE_MAX_HEIGHT 2.4 //meters
#define CANDIDATE_MIN_DENSITY 0.3 
#define DELTAXZ 0.5
#define DELTAY 1.0

//classification params
#define HOD_CELL_SIZE 16
#define HOD_ORIENT_BINS 18
#define HOD_BLOCK_SIZE 2
#define HOD_BLOCK_SHIFT 1

//candidate class params
#define F_H 58//*3.141592653589793/180)
#define F_V 45//*3.141592653589793/180)
#define F_HPELS 580
#define PEL_WIDTH 640
#define PEL_HEIGHT 480
#define RANSACK 20
#define EPSILON 0.1
#define HEURISTIC_HEIGHT
#define HEURISTIC_WIDTH

//Training params
#define DATADIR "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/"
#define MODEL "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/model/model0"
#define SVMDIR "/home/sam/cued-masters/src/SVMLight/"
#define FRAMESDIR "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/bag_frames/"
#define ANNOTDIR "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/annotations/"
#define COMPLOG "complete/log.txt"
#define COMPBOXES "complete/boxes"
#define COMPDESC "complete/descriptors"




#endif
