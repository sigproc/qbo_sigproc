#! /bin/bash
echo "recording:"
echo "image_raw, camera_info, rgb and depth, /tf"
rosbag record /camera/depth/image_raw /camera/rgb/image_raw /camera/depth/camera_info /camera/rgb/camera_info /tf

