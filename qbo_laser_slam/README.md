# Q.bo Package for performing SLAM by fg295: qbo_laser_slam

This package contains applications to perform laser based SLAM on the Q.Bo  
#### To use:  

First, build the package. In a terminal, navigate to the root of your catkin workspace (eg. ~/cued-masters) and type:
```console
$ catkin_make
```  

To run the system and perform SLAM:
```console
$ roslaunch qbo_laser_slam mapping.launch
```  

Or to perform Navigation:
```console
$ roslaunch qbo_laser_slam navigation.launch
```  

Both of which should be run on the robot (e.g. ssh into the robot and then run the previous commands).

The process can then be visualised in rviz on an external system:
```console
$ rosrun rviz rviz
```  
