# Q.bo Point Cloud Library Package: qbo_pcl

This package contains various point cloud applications.

## qbo_pcl_accumulate:

Takes a point cloud published on the topic "camera/depth/transformed_points" as input and appends it to an existing point cloud. To avoid memory catastrophe, we filter the accumulated points with a voxel filter after they have been added. The resulting accumulated and filtered point cloud is then published to the topic "map/point_cloud".

## qbo_pcl_transform:

Transforms point clouds published on the "camera/depth/points" topic into the /odom frame. Publishes the transformed point cloud on the "camera/depth/transformed_points" topic.

## qbo_pcl_decimator:

Filters point clouds on the "camera/depth/transformed_points" topic with a voxel grid filter and publishes them on the "camera/depth/decimated_points" topic.

