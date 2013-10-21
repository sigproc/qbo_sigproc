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

ros::Publisher pub;

void decimate_pc(const sensor_msgs::PointCloud2 &rich_ros_pc){

	pcl::PCLPointCloud2::Ptr rich_pcl_pc (new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr sparse_pcl_pc (new pcl::PCLPointCloud2());
	sensor_msgs::PointCloud2 sparse_ros_pc;

	pcl_conversions::toPCL(rich_ros_pc, *rich_pcl_pc);

  	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  	sor.setInputCloud (rich_pcl_pc);
  	sor.setLeafSize (0.01f, 0.01f, 0.01f);
  	sor.filter(*sparse_pcl_pc);

  	std::cerr << "PointCloud before filtering: " << rich_pcl_pc->width * rich_pcl_pc->height 
    << " data points (" << pcl::getFieldsList (*rich_pcl_pc) << ")." << std::endl;

    std::cerr << "PointCloud after filtering: " << sparse_pcl_pc->width * sparse_pcl_pc->height 
       << " data points (" << pcl::getFieldsList (*sparse_pcl_pc) << ")."<< std::endl;

  	pcl_conversions::fromPCL(*sparse_pcl_pc, sparse_ros_pc);

  	pub.publish(sparse_ros_pc);
}

int main(int argc, char **argv){
	ros::init(argc,argv, "point_cloud_decimator");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("camera/depth/points", 1, decimate_pc);

	pub = node.advertise<sensor_msgs::PointCloud2> ("camera/depth/decimated_points",1);

	ros::spin();
}