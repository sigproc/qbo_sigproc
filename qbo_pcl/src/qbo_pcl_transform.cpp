#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
// PCL specifics:
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
// tf specifics
#include <tf/transform_listener.h>
#include <iostream>
#include <string>


class Transformer
{
public:
	Transformer(std::string target_frame="/odom",std::string source_pc_topic="camera/depth/points", std::string target_pc_topic="camera/depth/transformed_points")
	{
		_target_frame = target_frame;
		_sub = _node.subscribe(source_pc_topic, 1, &Transformer::transformAndPublishPC,this);
		_pub = _node.advertise<sensor_msgs::PointCloud2> (target_pc_topic,1);
		ROS_INFO("Transformer initialized");
	}
	~Transformer(){
	}

	void transformAndPublishPC(const sensor_msgs::PointCloud2&);

private:
	std::string _target_frame;
	ros::NodeHandle _node;
	ros::Publisher _pub; 
	tf::TransformListener _transform_listener;
	ros::Subscriber _sub;
};

void Transformer::transformAndPublishPC(const sensor_msgs::PointCloud2 &inputCloud){
	ROS_INFO("Transforming PC");
	sensor_msgs::PointCloud2 output;
	std::cout << inputCloud.header.frame_id << std::endl;
	output.header = inputCloud.header;
	//Get the transform matrix and do transformation here

	if(!pcl_ros::transformPointCloud(_target_frame, inputCloud, output, _transform_listener)){
		ROS_ERROR("Something went wrong when transforming PC");
		return;
	}
	//Publish transformed cloud here
	std::cout << output.header.frame_id << std::endl;
	_pub.publish(output);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "pointcloud_transformer");

	Transformer myTransformer;

	ros::spin();
}
