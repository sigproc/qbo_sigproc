#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "Eigen/Core"
#include <Eigen/Geometry>
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
      : map_ptr(new pcl::PointCloud<pcl::PointXYZ>), map_updates(0)
    {

      cam_sub = nh.subscribe("camera/depth/decimated_points", 1, &Registrator::find_icp_transform, this);

      map_sub = nh.subscribe("map/point_cloud", 1, &Registrator::update_map, this);

      ROS_INFO("Registrator initialized");
    }

    pcl::PointCloud<pcl::PointXYZ> convertROSPointCloud2toPCL(const sensor_msgs::PointCloud2& ros_pc);
  	void find_icp_transform(const sensor_msgs::PointCloud2 &cam_points);
    void update_map(const sensor_msgs::PointCloud2 &map);
    void publish_visual_odometry(tf::Transform);

  private:
    tf::Quaternion mat_to_quat( Eigen::Matrix4f& );
  	ros::Subscriber cam_sub;
    ros::Subscriber map_sub;
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr;
    tf::TransformBroadcaster br;
    int map_updates;

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


  Eigen::Matrix4d md = icp.getFinalTransformation();;
  
  //Eigen::Quaternion<tfScalar> rotation = md.topLeftCorner<3,3>();
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(md(0,3),md(1,3) ,md(2,3) ) );
  //transform.setRotation(rotation);
  publish_visual_odometry(transform);
}

void Registrator::update_map(const sensor_msgs::PointCloud2 &map){
  //Lets update the map 10 times before we try to locate ourselves in it
  if(map_updates<50){
    *map_ptr = convertROSPointCloud2toPCL(map);
    ROS_INFO("Map Updated!");
    map_updates++;
  }
}

void Registrator::publish_visual_odometry(tf::Transform transform){
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/robot_guess"));
}

// create a unit quaternion
tf::Quaternion Registrator::mat_to_quat( Eigen::Matrix4f& m ) {
    tf::Quaternion rotation;
    float tr = m(0,0) + m(1,1) + m(2,2); // trace of martix
    if (tr > 0.0){
        tf::Quaternion rotation( m(1,2) - m(2,1), m(2,0) - m(0,2), m(0,1) - m(1,0), tr+1.0 );
        
    } else if( (m(0,0) > m(1,1) ) && ( m(0,0) > m(2,2)) ) {
        tf::Quaternion rotation( 1.0f + m(0,0) - m(1,1) - m(2,2), m(1,0) + m(0,1),
        m(2,0) + m(0,2), m(1,2) - m(2,1) );
        
    }else if ( m(1,1) > m(2,2) ){
        tf::Quaternion rotation( m(1,0) + m(0,1), 1.0f + m(1,1) - m(0,0) - m(2,2),
        m(2,1) + m(1,2), m(2,0) - m(0,2) ); 
        
    }else {
        tf::Quaternion rotation( m(2,0) + m(0,2), m(2,1) + m(1,2),
        1.0f + m(2,2) - m(0,0) - m(1,1), m(0,1) - m(1,0) );
       
    }
    rotation.normalize();
    return rotation;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "registration_node");
	Registrator reg;

	ros::spin();
}