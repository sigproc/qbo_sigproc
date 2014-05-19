#ifndef CONFIG
#define CONFIG

//#define QBO

//Select which images to show from running
#define SHOWDEPTHIM true
#define SHOWDEPTHSEG false
#define SHOWNORMALSEG false
#define SHOWNORMALIM false
#define SHOWJOINTSEG true
#define SHOWCANDIDATES true
#define SHOWBOXES true//*/

/*#define SHOWDEPTHIM true
#define SHOWDEPTHSEG true
#define SHOWNORMALSEG true
#define SHOWNORMALIM true
#define SHOWJOINTSEG true
#define SHOWCANDIDATES false
#define SHOWBOXES false //*/


//define system parameters
#define SIGMA 0.5
#define ALPHA 8
#define ESS 13
#define KDEPTH 6
#define KNORMAL 1.6
#define MIN_SIZE 30
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
#define MIN_INLIER_FRACTION 0.5 // for rejecting non-planar regions
#define VALID_VALUE 0.000001
#define CANDIDATE_MIN_WIDTH 0.3 //meters
#define CANDIDATE_MAX_WIDTH 1.5 //meters
#define CANDIDATE_MIN_HEIGHT 1.0 //meters
#define CANDIDATE_MAX_HEIGHT 1.7 //meters
#define CANDIDATE_MIN_DENSITY 0.2
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
/*#ifdef QBO
	#define DATADIR "/home/qbo/cued-masters/src/qbo_sigproc/human_detection/data/"
	#define MODEL "/home/qbo/cued-masters/src/qbo_sigproc/human_detection/data/model/model0"
	#define SVMDIR "/home/qbo/cued-masters/src/qbo_sigproc/human_detection/SVMLight/"
	#define FRAMESDIR "/home/qbo/cued-masters/src/qbo_sigproc/human_detection/data/bag_frames/"
	#define ANNOTDIR "/home/qbo/cued-masters/src/qbo_sigproc/human_detection/data/annotations/"
*/
//#else

	#define DATADIR "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/"	
	#define MODEL "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/model/raws-12-15-17-5-2014"
	#define SVMDIR "/home/sam/cued-masters/src/qbo_sigproc/human_detection/SVMLight/"
	#define FRAMESDIR "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/bag_frames/"
	#define ANNOTDIR "/home/sam/cued-masters/src/qbo_sigproc/human_detection/data/annotations/"

//#endif

//directory definitions
#define FRAMES "frames/"
#define TAGS "tags/"
#define DESC "descriptors/"
#define COMP "complete/"
#define METRIC "metric"
#define SEG "segmented"
#define FILLED "filled"
#define BOX_HITS "box_hits"
#define TAGLOG "Tagging_log"
#define CLASSIFLOG "Classification_log"
#define SEGTEST "seg_test"
#define TIMETEST "timings"

//training params
#define CAND_OVRLP_TRUE 0.4


#define COMPLOG "complete/log.txt"
#define COMPBOXES "complete/boxes"
#define COMPDESC "complete/descriptors"


//Testing Params
#define SEGIMRAW "frame_00366.jsc68"
#define SEGIMBED "frame_00006.jsc68"
#define SEGIMCOR "frame_00006.jsc68"
#define SEGIMKIT "frame_00006.jsc68"
#define SEGIMLIV "frame_00006.jsc68"






#endif
