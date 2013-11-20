#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>



//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//prototype
int segment(float sigma, float k, int min_size, cv::Mat image);

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

/*
 
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
 
//prototype
int segment(float sigma, float k, int min_size, const char* input, const char* output);


static void savePPM(image<rgb> *im, const char *name) {
  int width = im->width();
  int height = im->height();
  std::ofstream file(name, std::ios::out | std::ios::binary);

  file << "P6\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
  file.write((char *)imPtr(im, 0, 0), width * height * sizeof(rgb));
}

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
 
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 
	const char* filename = "tmpIm.ppm";
	float sigma = 0.5;
	float k = 500;
	int min_size = 20;

	//params for conversion
	int width, height;
	width = cv_ptr->image.size().width;
	height = cv_ptr->image.size().height;
	uchar rval = 0, gval=0, bval=0;

	//direct conversion from Mat to image<rgb>
	image<rgb> *inputIm = new image<rgb>(width,height);
	image<rgb> *outputIm = new image<rgb>(width,height);

	//Now scan through 
	for (int y=0; y < height; y++){
		for(int x=0; x<width; x++){
			//get values from
			cv::Vec3b intensity = cv_ptr->image.at<cv::Vec3b>(y,x);
			rval = intensity.val[2];
			gval = intensity.val[1];
			bval = intensity.val[0];
			imRef(inputIm,x,y).r = rval;
			imRef(inputIm,x,y).g = gval;
			imRef(inputIm,x,y).b = bval;
		}
	}


	int num_ccs; 
	outputIm = segment_image(inputIm, sigma, k, min_size, &num_ccs);

   	//replace data in message
    //Go through all the rows
    for(int y=0; y<cv_ptr->image.rows; y++)
    {
        //Go through all the columns
        for(int x=0; x<cv_ptr->image.cols; x++)
        {
            //assume red is 2
			cv_ptr->image.data[y*cv_ptr->image.rows*4+x*3 + 2] = imRef(outputIm,x,y).r;
			//green is 1
			cv_ptr->image.data[y*cv_ptr->image.rows*4+x*3 + 1] = imRef(outputIm,x,y).g;
			//blue is 0
			cv_ptr->image.data[y*cv_ptr->image.rows*4+x*3 + 0] = imRef(outputIm,x,y).b;

        }
    }
     
 
    //Display the image using OpenCV
    cv::imshow(WINDOW, cv_ptr->image);

    cv::waitKey(3);
        pub.publish(cv_ptr->toImageMsg());
}
 

int main(int argc, char **argv)
{
    
        ros::init(argc, argv, "image_processor");
  
        ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
        image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
        pub = it.advertise("/camera/rgb/human_segmentation", 1);
        ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}*/ 


//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 
	const char* filename = "tmpIm.ppm";
	float sigma = 0.5;
	float k = 500;
	int min_size = 20;

	//cheap hack: save image as .ppm
	//cv::imwrite(filename, cv_ptr->image);
	//std::cout << "written file" << std::endl;
	//load .ppm file into segment algorithm and save in .ppm tmp file
	//segment(sigma,k,min_size,cv_ptr->image);
	//now load back into cv_ptr: 1=> color image
	//cv::Mat image = cv::imread(filename, 1);
	//cv_ptr->image = image.data;



/*
   //copy to message ptr
    //Go through all the rows
    for(int i=0; i<cv_ptr->image.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<cv_ptr->image.cols; j++)
        {
            //Go through all the channels (b, g, r)
            for(int k=0; k<cv_ptr->image.channels(); k++)
            {
                //Invert the image by subtracting image data from 255              
                cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = image.data[i*image.rows*4+j*3 + k];
            }
        }
    }
  */   
 	double minval, maxval;
    cv::minMaxIdx(cv_ptr->image, &minval, &maxval);
    std::cout << "Minval: " << minval << std::endl;
    std::cout << "Maxval: " << maxval << std::endl;


    //cv::Mat *output_Im = new cv::Mat();
    cv_ptr->image.convertTo(cv_ptr->image, CV_8UC1,255.0/maxval); //32.0 is just a number which works. Need to understand which is the best number
    //cv::applyColorMap(cv_ptr->image, cv_ptr->image, cv::COLORMAP_JET);


    //Display the image using OpenCV
    cv::imshow(WINDOW, cv_ptr->image);

    cv::waitKey(3);
        pub.publish(cv_ptr->toImageMsg());
}
 
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
    
        ros::init(argc, argv, "image_processor");
  
        ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
        image_transport::Subscriber sub = it.subscribe("/camera/depth/image", 1, imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
        pub = it.advertise("/camera/depth/human_segmentation", 1);
        ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}
