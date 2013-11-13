# Simple Naviagtion Package
	
 A package called kdl has to be installed to use this node. It handles operations using quaternions which is how the orientation of the robot is stored. Also it is not supported on hydro, so i had to install it manually.
 To install go to your "/cuedmasters/src" directory and run 

 ```console
 $ git clone http://git.mech.kuleuven.be/robotics/orocos_kinematics_dynamics.git
 ```

 then run 

 ```console
 $ rosmake kdl
 ```


 This should then install kdl and pykdl to the correct place for you to run the node.





 Package containing a simple node which monitors two topics one for the current pose and one for a goal pose and then sends commands to the Qbo to navigate from one to the other.

 To use : 

 startup the Qbo using qbo.launch

 run 

 ```console
 	$ roslaunch simple_naviagtion qbo_nav.launch
 ```
 open Rviz 

 In Rviz set a 2d nav goal (make sure the nav goal topic is set to /goal_pose)
 


