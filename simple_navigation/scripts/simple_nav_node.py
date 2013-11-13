#!/usr/bin/env python
import sys
import math
import rospy
import roslib
roslib.load_manifest("python_orocos_kdl")
import PyKDL
from geometry_msgs.msg import PoseStamped, Twist, Transform


class simple_nav_node:

	def __init__(self): 
		"Constructor - initialises the topic subscriptions and publisher and some variables which need to be predifened for the while loop in navgate to start."
		self.goal_pose = rospy.Subscriber("/goal_pose",PoseStamped,self.callback1) 
		self.current_pose = rospy.Subscriber("/current_pose",PoseStamped,self.callback2)
		self.move_command = rospy.Publisher("/cmd_vel", Twist)
		self.speed = 0.5
		self.current_pose = None
		self.goal_pose = None
		self.in_orientation = False
		self.in_postition = False
		self.pi = 3.14159265
		

	def callback1(self,data):
		#rospy.loginfo("Goal Recieved")
		self.goal_pose = data.pose
		

	def callback2(self,data):
		#rospy.loginfo("Pose Recieved")
		self.current_pose = data.pose
		
		

	def navigate(self):
		"Called when both the current pose and a goal pose have been recieved, runs until robot is close enough to the goal pose"
		while (not self.in_orientation) & (not self.in_postition) : 
			# Sets up the path from the current pose to the goal pose
			path_vector = [0,0]
			path_vector[0] = self.goal_pose.position.x - self.current_pose.position.x
			path_vector[1] = self.goal_pose.position.y - self.current_pose.position.y
			
			# Finds the current angle about teh z axis from the quaternions representation recieved from the topic
			current_orientation = PyKDL.Rotation.Quaternion(self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w)
			current_angle = current_orientation.GetRPY()[2]
			rospy.loginfo("Current Angle = "+str(current_angle))

			#find the difference between the current heading and the path_vector angle
			goal_angle = math.atan2(path_vector[1],path_vector[0])
			#rospy.loginfo("Goal Angle = "+ str(goal_angle))
			facing_difference = goal_angle - current_angle
			#rospy.loginfo(facing_difference)
			
			#find the difference between the current heading and the orientation of the goal state
			goal_orientation = PyKDL.Rotation.Quaternion(self.goal_pose.orientation.x, self.goal_pose.orientation.y, self.goal_pose.orientation.z, self.goal_pose.orientation.w)
			c = goal_orientation * current_orientation.Inverse()
			turn_needed = c.GetRPY()[2] # get roll pitch yaw result

			#checks the state of the robot
			#rospy.loginfo("Turn Needed = "+ str(turn_needed))
			if (abs(path_vector[0]) < 0.1) & (abs(path_vector[1]) < 0.1):
				in_postition = True
			else:
				in_postition = False
			#rospy.loginfo("In position = "+ str(in_postition) + "Path vector is " +str(path_vector))

			if (abs(path_vector[0]) < 0.5) & (abs(path_vector[1]) < 0.5):
				nearly_in_postition = True
			else:
				nearly_in_postition = False


			if abs(facing_difference) < 0.3:
				facing_goal = True
			else:
				facing_goal = False	 
			#rospy.loginfo("facing_goal = "+ str(facing_goal))

			if abs(turn_needed) < 0.05:
				in_orientation = True
			else:
				in_orientation = False	
			#rospy.loginfo("in_orientation = "+ str(in_orientation))

			#The code to send a turn command to the robot"
			
			if (in_postition & in_orientation): #4. Stop
				rospy.loginfo("YOU HAVE ARRIVED AT YOUR DESTINATION!!!")
				break
			elif (not in_orientation) & in_postition: #3. turn to face the correct way 
				"The code to make the robot face the correct direction"
				twist3 = Twist()
				twist3.angular.x = 0; twist3.angular.y = 0; twist3.angular.z = math.copysign(0.1,turn_needed)+ turn_needed*self.speed
				self.move_command.publish(twist3) 
				rospy.loginfo("Turning into correct orientation")
			elif facing_goal & (not in_postition ): #2. Drive forwards
				"The code to send a drive forwards command to the robot"
				twist2 = Twist()
				twist2.linear.x = self.speed*0.5; twist2.linear.y = 0; twist2.linear.z = 0
				self.move_command.publish(twist2) 
				rospy.loginfo("Driving towards destination")
			elif facing_goal & (not nearly_in_postition ): #2. Drive forwards
				"The code to send a drive forwards command to the robot"
				twist2 = Twist()
				twist2.linear.x = self.speed; twist2.linear.y = 0; twist2.linear.z = 0
				self.move_command.publish(twist2) 
				rospy.loginfo("Driving towards destination")
			elif not facing_goal: #1. Turn towards goal
				twist1 = Twist()
				twist1.angular.x = 0; twist1.angular.y = 0; twist1.angular.z = math.copysign(0.1,facing_difference)+  facing_difference*self.speed
				self.move_command.publish(twist1)
				rospy.loginfo("Turning to face goal")

		# stop the robot -- Possibly redundant 
		twist4 = Twist()
		self.move_command.publish(twist4)
		


def main(args):
	snn = simple_nav_node()
	rospy.init_node('simple_nav_node', anonymous=True)
	
	while (not rospy.is_shutdown()):

		while (snn.current_pose == None):
			rospy.loginfo("Waiting for Pose")
		while (snn.goal_pose == None):
			rospy.loginfo("Waiting for Goal")
			
		snn.navigate()

		



if __name__ == '__main__':
	main(sys.argv)	
