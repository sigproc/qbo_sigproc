#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
// PCL specifics:
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// Other
#include <iostream>


/*
 Decimator filters point clouds on the "camera/depth/transformed_points" topic with a voxel grid filter and 
 publishes them on the "camera/depth/decimated_points" topic.
*/
class Decimator
{
public:
  Decimator(const ros::NodeHandle& nh, 
                   const ros::NodeHandle& nh_private):
  _nh(nh),
  _nh_private(nh_private)
  {
    initParams();
    _sub = _nh.subscribe(_input_topic, 1, &Decimator::decimate_pc,this);
    _pub = _nh.advertise<sensor_msgs::PointCloud2> (_output_topic,1);
    ROS_INFO("Decimator initialized");
  }
  ~Decimator(){}

  private:
  void initParams(void);
  void decimate_pc(const sensor_msgs::PointCloud2&);
  double _leaf_size;
  std::string _input_topic;
  std::string _output_topic;
  ros::NodeHandle _nh;                ///< the public nodehandle
  ros::NodeHandle _nh_private;        ///< the private nodehandle
  ros::Publisher _pub;
  ros::Subscriber _sub;
};

void Decimator::initParams(void){
  if (!_nh_private.getParam ("leaf_size", _leaf_size)){
    _leaf_size=0.05;
    ROS_WARN("Need to set voxel grid leaf_size argument! Setting leaf size to 0.05");
  }
  if (!_nh_private.getParam ("input_topic", _input_topic)){
      _input_topic = "camera/depth/transformed_points";
      ROS_WARN("Need to set input_topic argument! Setting input_topic to camera/depth/transformed_points");
  }
  if (!_nh_private.getParam ("output_topic", _output_topic))
  {
    _output_topic = "camera/depth/decimated_points";  
    ROS_WARN("Need to set output_topic argument! Setting input_topic to camera/depth/decimated_points");
  }
}

void Decimator::decimate_pc(const sensor_msgs::PointCloud2 &rich_ros_pc){

	pcl::PCLPointCloud2::Ptr rich_pcl_pc (new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr sparse_pcl_pc (new pcl::PCLPointCloud2());
	sensor_msgs::PointCloud2 sparse_ros_pc;

	pcl_conversions::toPCL(rich_ros_pc, *rich_pcl_pc);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (rich_pcl_pc);
  sor.setLeafSize (_leaf_size, _leaf_size, _leaf_size);
  sor.filter(*sparse_pcl_pc);

 	std::cout << "PointCloud before filtering: " << rich_pcl_pc->width * rich_pcl_pc->height 
  << " data points (" << pcl::getFieldsList (*rich_pcl_pc) << ")." << std::endl;

  std::cout << "PointCloud after filtering: " << sparse_pcl_pc->width * sparse_pcl_pc->height 
  << " data points (" << pcl::getFieldsList (*sparse_pcl_pc) << ")."<< std::endl;

	pcl_conversions::fromPCL(*sparse_pcl_pc, sparse_ros_pc);

 	_pub.publish(sparse_ros_pc);
}

int main(int argc, char **argv){
	ros::init(argc,argv, "point_cloud_decimator");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  Decimator myDecimator(nh,nh_private);

	ros::spin();
  return 0;
}