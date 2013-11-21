#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
// PCL specifics:
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>

/*Accumulator takes a point cloud published on the topic "camera/depth/transformed_points" as input
 and appends it to an existing point cloud. To avoid memory catastrophe, we filter the 
 accumulated points with a voxel filter after they have been added. The resulting accumulated 
 and filtered point cloud is then published to the topic "map/point_cloud".*/
class Accumulator
{
public:
	Accumulator(const ros::NodeHandle& nh, 
                   const ros::NodeHandle& nh_private):
  	_nh(nh),
  	_nh_private(nh_private),
	_accumulated_points_ptr(new pcl::PointCloud<pcl::PointXYZ>)
	{
		initParams();
		_sub = _nh.subscribe(_input_topic, 1, &Accumulator::accumulate_pc,this);
		_pub = _nh.advertise<sensor_msgs::PointCloud2> (_output_topic,1);
		ROS_INFO("Accumulator initialized");
	}

	~Accumulator(){}

private:
	void initParams(void);
	void accumulate_pc(const sensor_msgs::PointCloud2 &ros_pc);
	ros::Subscriber _sub;
	ros::NodeHandle _nh;                ///< the public nodehandle
 	ros::NodeHandle _nh_private;        ///< the private nodehandle
	pcl::PointCloud<pcl::PointXYZ>::Ptr _accumulated_points_ptr;
	ros::Publisher _pub;
	std::string _input_topic;
  	std::string _output_topic;
  	double _leaf_size;
};

void Accumulator::initParams(void){
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
    _output_topic = "map/point_cloud";  
    ROS_WARN("Need to set output_topic argument! Setting input_topic to map/point_cloud");
  }
}

void Accumulator::accumulate_pc(const sensor_msgs::PointCloud2 &ros_pc){

	pcl::PCLPointCloud2 pcl_pc2; 
	pcl_conversions::toPCL(ros_pc, pcl_pc2);

	//Convert to PointCloud
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	pcl::fromPCLPointCloud2(pcl_pc2, pcl_pc);

	//Stitch it all up
	*_accumulated_points_ptr += pcl_pc;
	
	//Convert & Filter it!

  	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (_accumulated_points_ptr);
  	sor.setLeafSize (_leaf_size, _leaf_size, _leaf_size);
  	pcl::PointCloud<pcl::PointXYZ> temp;
  	sor.filter(temp);

  	*_accumulated_points_ptr = temp;

    //Publish it

    // temporary PointCloud2 intermediary
    pcl::PCLPointCloud2 tmp_pc;

    // Convert fused from PCL native type to ROS
    pcl::toPCLPointCloud2(*_accumulated_points_ptr, tmp_pc);
    
    sensor_msgs::PointCloud2 published_pc;
    pcl_conversions::fromPCL(tmp_pc, published_pc);

    //Both messages are in the odom frame, so this should be fine
    published_pc.header = ros_pc.header;

    // Publish the data
    _pub.publish(published_pc);
}


int main(int argc, char **argv){
	ros::init(argc,argv, "point_cloud_accumulator");

	ros::NodeHandle nh;
  	ros::NodeHandle nh_private("~");

	Accumulator myAccumulator(nh,nh_private);

	ros::spin();
}