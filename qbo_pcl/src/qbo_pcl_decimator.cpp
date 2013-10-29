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



class Decimator
{
public:
  Decimator(std::string input_pc_topic="camera/depth/transformed_points",std::string output_pc_topic="camera/depth/decimated_points"){
  _sub = _node.subscribe(input_pc_topic, 1, &Decimator::decimate_pc,this);
  _pub = _node.advertise<sensor_msgs::PointCloud2> (output_pc_topic,1);
  ROS_INFO("Decimator initialized");
  }
  ~Decimator(){}

  private:
  void decimate_pc(const sensor_msgs::PointCloud2&);
  ros::NodeHandle _node;
  ros::Publisher _pub;
  ros::Subscriber _sub;
};

void Decimator::decimate_pc(const sensor_msgs::PointCloud2 &rich_ros_pc){

	pcl::PCLPointCloud2::Ptr rich_pcl_pc (new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr sparse_pcl_pc (new pcl::PCLPointCloud2());
	sensor_msgs::PointCloud2 sparse_ros_pc;

	pcl_conversions::toPCL(rich_ros_pc, *rich_pcl_pc);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (rich_pcl_pc);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
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

  Decimator myDecimator;

	ros::spin();
}