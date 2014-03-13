#! /bin/bash
echo "roslaunch human_detection playbag.launch user:=$USER"
roslaunch human_detection seebag.launch user:=$USER bag_file:=$1
