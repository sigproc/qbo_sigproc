#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
// PCL specifics:
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// tf specifics
#include <tf/transform_listener.h>
#include <iostream>

//Need to be able to listen for both point cloud and tf frame

//I'm going to start by just trying to write the listeners for those,
//which will read back the messages to the console. I'll start with
//the tf listener and move on to the pcl listener


//Instantiating the publisher that publishes the transformed pointcloud;
ros::Publisher pub; 

void transform_pc(const sensor_msgs::PointCloud2 inputCloud){
	//Get the transform matrix and do transformation here

	sensor_msgs::PointCloud2 output = inputCloud;
	//Publish transformed cloud here
	pub.publish(output);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "listener");

	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("camera/depth/points", 1, transform_pc);

	pub = node.advertise<sensor_msgs::PointCloud2> ("camera/transformed_depth_points",1);

	ros::spin();
}
