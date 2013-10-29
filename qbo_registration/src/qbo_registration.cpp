#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <tf/tf.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>


class Registrator
{
  public:
    Registrator()
      : map_ptr(new pcl::PointCloud<pcl::PointXYZ>)
    {

      cam_sub = nh.subscribe("camera/depth/decimated_points", 1, &Registrator::find_icp_transform, this);

      map_sub = nh.subscribe("map/point_cloud", 1, &Registrator::update_map, this);
      //tf_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

      ROS_INFO("Registrator initialized");
    }

    pcl::PointCloud<pcl::PointXYZ> convertROSPointCloud2toPCL(const sensor_msgs::PointCloud2& ros_pc);
  	void find_icp_transform(const sensor_msgs::PointCloud2 &cam_points);
    void update_map(const sensor_msgs::PointCloud2 &map);
    void publish_visual_odometry(void);

  private:
  	ros::Subscriber cam_sub;
    ros::Subscriber map_sub;
    ros::NodeHandle nh;
    ros::Publisher tf_pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr;

};

pcl::PointCloud<pcl::PointXYZ> Registrator::convertROSPointCloud2toPCL(const sensor_msgs::PointCloud2& ros_pc){
  //Convert to PointCloud2
  pcl::PCLPointCloud2 pcl_pc2; 
  pcl_conversions::toPCL(ros_pc, pcl_pc2);

  //Convert to PointCloud
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  pcl::fromPCLPointCloud2(pcl_pc2, pcl_pc);

  return pcl_pc;
}

void Registrator::find_icp_transform(const sensor_msgs::PointCloud2 &cam_points){
  ROS_INFO("Finding ICP transform");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_points_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  *cam_points_ptr = convertROSPointCloud2toPCL(cam_points);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cam_points_ptr);
  icp.setInputTarget(map_ptr);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << icp.getFinalTransformation() << std::endl;
}

void Registrator::update_map(const sensor_msgs::PointCloud2 &map){
  *map_ptr = convertROSPointCloud2toPCL(map);
  ROS_INFO("Map Updated!");
}

void Registrator::publish_visual_odometry(){

}


int main(int argc, char **argv)
{
	ros::init(argc,argv, "registration_node");
	Registrator reg;

	ros::spin();
}