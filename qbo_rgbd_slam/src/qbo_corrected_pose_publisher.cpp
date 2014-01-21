#include "ros/ros.h"
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

class CorrectedPosePublisher
{
public:
	CorrectedPosePublisher(const ros::NodeHandle& nh, 
                   const ros::NodeHandle& nh_private):
  	nh_(nh),
  	nh_private_(nh_private)
	{
		initParams();
		odom_sub_ = nh_.subscribe(odom_topic_, 1, &CorrectedPosePublisher::publish_corrected_pose,this);
		pose_correction_sub_ = nh_.subscribe(correction_topic_, 1, &CorrectedPosePublisher::update_pose_correction,this);
		ROS_INFO("Corrected Pose Published initialized");
	}

	~CorrectedPosePublisher(){}

private:
	void initParams(void);
  void publish_corrected_pose(const nav_msgs::Odometry & odom_msg);
  void update_pose_correction(const geometry_msgs::Transform & pose_correction_msg);
	ros::Subscriber odom_sub_;
  ros::Subscriber pose_correction_sub_;
	ros::NodeHandle nh_;                ///< the public nodehandle
 	ros::NodeHandle nh_private_;        ///< the private nodehandle
	tf::TransformBroadcaster pose_broadcaster_;
	std::string odom_topic_;
  std::string correction_topic_;
  std::string fixed_frame_id_;
  std::string child_frame_id_;
  tf::Transform correction_tf_;
};

void CorrectedPosePublisher::initParams(void){
  if (!nh_private_.getParam ("odom_topic", odom_topic_)){
    odom_topic_="odom";
    ROS_WARN("Need to set odom_topic argument! Setting odom_topic to odom");
  }
  if (!nh_private_.getParam ("correction_topic", correction_topic_)){
    correction_topic_ = "pose_correction_tfs";
    ROS_WARN("Need to set correction_topic argument! Setting correction_topic to pose_correction_tfs");
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

  correction_tf_.setIdentity(); 

}

void CorrectedPosePublisher::publish_corrected_pose(const nav_msgs::Odometry & odom_msg){
  tf::Transform correctedtf;
  tf::StampedTransform wheel_odom_tf;
  tf::poseMsgToTF(odom_msg.pose.pose, wheel_odom_tf);
  correctedtf.mult(correction_tf_,wheel_odom_tf);
  geometry_msgs::Transform correctedtfmsg;
  tf::transformTFToMsg(correctedtf,correctedtfmsg);

  geometry_msgs::TransformStamped odom_trans;
  
  odom_trans.header = odom_msg.header;
  odom_trans.header.frame_id = fixed_frame_id_;
  odom_trans.child_frame_id = child_frame_id_;
  odom_trans.transform = correctedtfmsg;
  
  pose_broadcaster_.sendTransform(odom_trans);
  ROS_INFO("Broadcast corrected fixed to moving frame transform! \n");
}

void CorrectedPosePublisher::update_pose_correction(const geometry_msgs::Transform & pose_correction_msg){
  tf::transformMsgToTF(pose_correction_msg,correction_tf_);
  ROS_INFO("Odom correction transform updated! \n");
}

int main(int argc, char **argv){
	ros::init(argc,argv, "corrected_pose_publisher");

	ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

	CorrectedPosePublisher myCorrectedPosePublisher(nh,nh_private);

	ros::spin();
}