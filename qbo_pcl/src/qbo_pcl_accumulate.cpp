#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
// PCL specifics:
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_points_ptr;
std::string base_frame_id;

void accumulate_pc(const sensor_msgs::PointCloud2 &ros_pc){

	pcl::PCLPointCloud2 pcl_pc2; 
	pcl_conversions::toPCL(ros_pc, pcl_pc2);

	//Convert to PointCloud
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	pcl::fromPCLPointCloud2(pcl_pc2, pcl_pc);

	//Stitch it all up
	*accumulated_points_ptr += pcl_pc;
	
	//Convert Filter it!


  	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (accumulated_points_ptr);
  	sor.setLeafSize (0.1f, 0.1f, 0.1f);
  	pcl::PointCloud<pcl::PointXYZ> temp;
  	sor.filter(temp);

  	*accumulated_points_ptr = temp;

    //Publish it

    // temporary PointCloud2 intermediary
    pcl::PCLPointCloud2 tmp_pc;

    // Convert fused from PCL native type to ROS
    pcl::toPCLPointCloud2(*accumulated_points_ptr, tmp_pc);
    
    sensor_msgs::PointCloud2 published_pc;
    pcl_conversions::fromPCL(tmp_pc, published_pc);

    published_pc.header = ros_pc.header;

    // Publish the data
    pub.publish(published_pc);
}


int main(int argc, char **argv){
	ros::init(argc,argv, "point_cloud_accumulator");
	ros::NodeHandle node;

	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(
		new pcl::PointCloud<pcl::PointXYZ>());
	accumulated_points_ptr = new_cloud;

	ros::Subscriber sub = node.subscribe("camera/depth/transformed_points", 1, accumulate_pc);

	pub = node.advertise<sensor_msgs::PointCloud2> ("camera/depth/accumulated_points",1);

	ros::spin();
}