#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>


namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("/camera/depth/human_segmentation", 1);
    image_sub_ = it_.subscribe("/camera/depth/image", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
	//Assign a new object to cv_ptrOut
    //cv_bridge::CvImagePtr cv_ptrOut(new cv_bridge::CvImage());
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //Look at values in input image
    //std::cout << "Input Image = " << std::endl << " " << cv_ptr->image << std::endl << std::endl;

    //find the min and max
    double minval, maxval;
    cv::minMaxIdx(cv_ptr->image, &minval, &maxval);
    std::cout << "Minval: " << minval << std::endl;
    std::cout << "Maxval: " << maxval << std::endl;


    //cv::Mat *output_Im = new cv::Mat();
    cv_ptr->image.convertTo(cv_ptr->image, CV_8UC1,255.0/maxval); //32.0 is just a number which works. Need to understand which is the best number
    cv::applyColorMap(cv_ptr->image, cv_ptr->image, cv::COLORMAP_JET);
    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);

    //Now put output_Im into a CvImagePtr

	//cv_ptrOut->header = cv_ptr->header;
	//cv_ptrOut->encoding = cv_ptr->encoding;
	//cv_ptrOut->image = *output_Im;

	//and convert ImagePtr into a msg for publishing    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_segmentation");
  ImageConverter ic;
  ros::spin();
  return 0;
}
