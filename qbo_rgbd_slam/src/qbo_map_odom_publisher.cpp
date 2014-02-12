#include "ros/ros.h"
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

class MapOdomPublisher
{
public:
	MapOdomPublisher(const ros::NodeHandle& nh, 
                   const ros::NodeHandle& nh_private):
  	nh_(nh),
  	nh_private_(nh_private)
	{
		initParams();
		pose_correction_sub_ = nh_.subscribe(correction_topic_, 1, &MapOdomPublisher::update_pose_correction,this);
		timer_ = nh.createTimer(ros::Duration(1/rate_),&MapOdomPublisher::publish_corrected_pose,this);
    ROS_INFO("Map odom publisher initialized");
	}

	~MapOdomPublisher(){}

private:
	void initParams(void);
  void publish_corrected_pose(const ros::TimerEvent& e);
  void update_pose_correction(const geometry_msgs::Transform & pose_correction_msg);
  ros::Subscriber pose_correction_sub_;
	ros::NodeHandle nh_;                ///< the public nodehandle
 	ros::NodeHandle nh_private_;        ///< the private nodehandle
	tf::TransformBroadcaster pose_broadcaster_;
  std::string correction_topic_;
  std::string fixed_frame_id_;
  std::string child_frame_id_;
  tf::StampedTransform correction_tf_;
  ros::Timer timer_;
  double rate_;
};

void MapOdomPublisher::initParams(void){
  if (!nh_private_.getParam ("rate", rate_)){
    rate_=60.0;
    ROS_WARN("Need to set rate argument! Setting rate to 15 Hz");
  }
  if (!nh_private_.getParam ("correction_topic", correction_topic_)){
    correction_topic_ = "pose_correction_tfs";
    ROS_WARN("Need to set correction_topic argument! Setting correction_topic to pose_correction_tfs");
  }
  if (!nh_private_.getParam ("fixed_frame_id", fixed_frame_id_))
  {
    fixed_frame_id_ = "/map";  
    ROS_WARN("Need to set fixed_frame_id argument! Setting fixed_frame_id to odom");
  }
   if (!nh_private_.getParam ("child_frame_id", child_frame_id_))
  {
    child_frame_id_ = "/odom";  
    ROS_WARN("Need to set child_frame_id argument! Setting child_frame_id to camera_link");
  }

  correction_tf_.setIdentity(); 

}

void MapOdomPublisher::publish_corrected_pose(const ros::TimerEvent& e){
  ROS_INFO("TimerEvent triggered \n");
  
  //apply correction to transform
  correction_tf_.stamp_ = ros::Time::now();
  correction_tf_.frame_id_ = fixed_frame_id_;
  correction_tf_.child_frame_id_ = child_frame_id_;

  geometry_msgs::TransformStamped odom_map_corrected_msg;
  tf::transformStampedTFToMsg(correction_tf_,odom_map_corrected_msg);

  pose_broadcaster_.sendTransform(odom_map_corrected_msg);
  ROS_INFO("Broadcast corrected fixed to moving frame transform! \n");
}

void MapOdomPublisher::update_pose_correction(const geometry_msgs::Transform & pose_correction_msg){
  tf::transformMsgToTF(pose_correction_msg,correction_tf_);
  ROS_INFO("Odom correction transform updated! \n");
}

int main(int argc, char **argv){
	ros::init(argc,argv, "map_odom_publisher");

	ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

	MapOdomPublisher myMapOdomPublisher(nh,nh_private);

	ros::spin();
}