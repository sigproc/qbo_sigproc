#include "ros/ros.h"
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

class OdomPublisher
{
public:
	OdomPublisher(const ros::NodeHandle& nh, 
                   const ros::NodeHandle& nh_private):
  	nh_(nh),
  	nh_private_(nh_private)
	{
		initParams();
		odom_sub_ = nh_.subscribe(odom_topic_, 1, &OdomPublisher::publish_odom,this);
		ROS_INFO("Corrected Pose Published initialized");
	}

	~OdomPublisher(){}

private:
	void initParams(void);
  void publish_odom(const nav_msgs::Odometry & odom_msg);
	ros::Subscriber odom_sub_;
	ros::NodeHandle nh_;                ///< the public nodehandle
 	ros::NodeHandle nh_private_;        ///< the private nodehandle
	tf::TransformBroadcaster pose_broadcaster_;
	std::string odom_topic_;
  std::string correction_topic_;
  std::string fixed_frame_id_;
  std::string child_frame_id_;
};

void OdomPublisher::initParams(void){
  if (!nh_private_.getParam ("odom_topic", odom_topic_)){
    odom_topic_="odom";
    ROS_WARN("Need to set odom_topic argument! Setting odom_topic to odom");
  }
  if (!nh_private_.getParam ("fixed_frame_id", fixed_frame_id_))
  {
    fixed_frame_id_ = "odom";  
    ROS_WARN("Need to set fixed_frame_id argument! Setting fixed_frame_id to odom");
  }
   if (!nh_private_.getParam ("child_frame_id", child_frame_id_))
  {
    child_frame_id_ = "base_footprint";  
    ROS_WARN("Need to set child_frame_id argument! Setting child_frame_id to base_footprint");
  }

}

void OdomPublisher::publish_odom(const nav_msgs::Odometry & odom_msg){

  geometry_msgs::TransformStamped odom_trans;
  
  odom_trans.header = odom_msg.header;
  odom_trans.header.frame_id = fixed_frame_id_;
  odom_trans.child_frame_id = child_frame_id_;
  tf::Transform odom_tf;
  tf::poseMsgToTF(odom_msg.pose.pose,odom_tf);
  tf::transformTFToMsg(odom_tf,odom_trans.transform);
  
  pose_broadcaster_.sendTransform(odom_trans);
}


int main(int argc, char **argv){
	ros::init(argc,argv, "odom_publisher");

	ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

	OdomPublisher myOdomPublisher(nh,nh_private);

	ros::spin();
}