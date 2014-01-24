#include "ros/ros.h"
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
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
		pose_correction_sub_ = nh_.subscribe(correction_topic_, 1, &CorrectedPosePublisher::update_pose_correction,this);
		timer_ = nh.createTimer(ros::Duration(1/rate_),&CorrectedPosePublisher::publish_corrected_pose,this);
    ROS_INFO("Corrected Pose Published initialized");
	}

	~CorrectedPosePublisher(){}

private:
	void initParams(void);
  void publish_corrected_pose(const ros::TimerEvent& e);
  void update_pose_correction(const geometry_msgs::Transform & pose_correction_msg);
  ros::Subscriber pose_correction_sub_;
	ros::NodeHandle nh_;                ///< the public nodehandle
 	ros::NodeHandle nh_private_;        ///< the private nodehandle
	tf::TransformBroadcaster pose_broadcaster_;
  tf::TransformListener tf_listener_;
  std::string correction_topic_;
  std::string fixed_frame_id_;
  std::string child_frame_id_;
  std::string corrected_child_frame_id_;
  tf::Transform correction_tf_;
  ros::Timer timer_;
  double rate_;
};

void CorrectedPosePublisher::initParams(void){
  if (!nh_private_.getParam ("rate", rate_)){
    rate_=10.0;
    ROS_WARN("Need to set rate argument! Setting rate to 15 Hz");
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
    child_frame_id_ = "camera_link";  
    ROS_WARN("Need to set child_frame_id argument! Setting child_frame_id to camera_link");
  }
  if (!nh_private_.getParam ("corrected_child_frame_id", corrected_child_frame_id_))
  {
    corrected_child_frame_id_ = "camera_link_corrected";  
    ROS_WARN("Need to set corrected_child_frame_id argument! Setting corrected_child_frame_id to camera_link_corrected");
  }

  correction_tf_.setIdentity(); 

}

void CorrectedPosePublisher::publish_corrected_pose(const ros::TimerEvent& e){
  ROS_INFO("TimerEvent triggered \n");
  //Get wheel odometry based transform between fixed frame and camera
  tf::StampedTransform fixed_to_cam_tf;
  tf::StampedTransform fixed_to_cam_corrected_tf;
  try{
    tf_listener_.waitForTransform(
      fixed_frame_id_, child_frame_id_, ros::Time(0), ros::Duration(5.0));
    tf_listener_.lookupTransform(
      fixed_frame_id_, child_frame_id_, ros::Time(0), fixed_to_cam_tf);  
  }
  catch(...)
  {
    ROS_INFO("Time out! \n");
    return;
  }
  
  //apply correction to transform
  fixed_to_cam_corrected_tf.stamp_ = fixed_to_cam_tf.stamp_;
  fixed_to_cam_corrected_tf.frame_id_ = fixed_frame_id_;
  fixed_to_cam_corrected_tf.child_frame_id_ = corrected_child_frame_id_;
  fixed_to_cam_corrected_tf.mult(correction_tf_,fixed_to_cam_tf);

  geometry_msgs::TransformStamped fixed_to_cam_corrected_msg;
  tf::transformStampedTFToMsg(fixed_to_cam_corrected_tf,fixed_to_cam_corrected_msg);

  pose_broadcaster_.sendTransform(fixed_to_cam_corrected_msg);
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