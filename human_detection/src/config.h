#ifndef CONFIG
#define CONFIG

//Select which images to show from running
#define SHOWDEPTHIM false
#define SHOWDEPTHSEG false
#define SHOWNORMALSEG false
#define SHOWNORMALIM false
#define SHOWJOINTSEG false
#define SHOWCANDIDATES true


//define system parameters
#define SIGMA 0.5
#define ALPHA 8
#define ESS 13
#define KDEPTH 5
#define KNORMAL 1
#define MIN_SIZE 15

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
#define CANDIDATE_MIN_WIDTH 0.2
#define CANDIDATE_MAX_WIDTH 1.2
#define CANDIDATE_MIN_HEIGHT 1.0
#define CANDIDATE_MAX_HEIGHT 2.4
#define CANDIDATE_MIN_DENSITY 0.3
#define DELTAXZ 0.5
#define DELTAY 1.0


//candidate class params
#define F_H 58*3.141592653589793/180
#define F_V 45*3.141592653589793/180
#define PEL_WIDTH 640
#define PEL_HEIGHT 480
#define RANSACK 20
#define EPSILON 0.1
#define HEURISTIC_HEIGHT
#define HEURISTIC_WIDTH




#endif
