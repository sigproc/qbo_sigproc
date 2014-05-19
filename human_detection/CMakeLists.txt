cmake_minimum_required(VERSION 2.8.3)
project(human_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES seg_depth
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET (CMAKE_CXX_FLAGS_DEBUG "-g -00")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(human_detection src/human_detection.cpp src/segment_depth/segment.cpp src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h src/classification/descriptor.h)
target_link_libraries(human_detection ${catkin_LIBRARIES})

add_executable(human_detection_2 src/human_detection_2.cpp src/segment_depth/segment.cpp src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h src/classification/descriptor.h)
target_link_libraries(human_detection_2 ${catkin_LIBRARIES})

#add_executable(driver_test src/driver_test.cpp src/segment_depth/segment.cpp src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h src/classification/descriptor.h)
#target_link_libraries(driver_test ${catkin_LIBRARIES})

add_executable(save_metric_frames src/save_metric_frames.cpp src/utilities/file_management.h )
target_link_libraries(save_metric_frames ${catkin_LIBRARIES})

#add_executable(hd_train src/hd_train.cpp src/segment_depth/segment.cpp src/utilities/file_management.h src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h src/classification/descriptor.h)
#target_link_libraries(hd_train ${catkin_LIBRARIES})

add_executable(hd_train_2 src/hd_train_2.cpp src/segment_depth/segment.cpp src/utilities/file_management.h src/utilities/frame.h  src/classification/descriptor.h  src/classification/box_tag.h src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h)
target_link_libraries(hd_train_2 ${catkin_LIBRARIES})

add_executable(hd_tag src/hd_tag.cpp src/segment_depth/segment.cpp src/utilities/file_management.h src/utilities/frame.h  src/classification/descriptor.h  src/classification/box_tag.h src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h)
target_link_libraries(hd_tag ${catkin_LIBRARIES})

add_executable(segmentation_test2 src/segmentation_test2.cpp src/segment_depth/segment.cpp src/utilities/file_management.h src/utilities/frame.h  src/classification/descriptor.h  src/classification/box_tag.h src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h)
target_link_libraries(segmentation_test2 ${catkin_LIBRARIES})

add_executable(pipeline_times src/pipeline_times.cpp src/segment_depth/segment.cpp src/utilities/file_management.h src/utilities/frame.h  src/classification/descriptor.h  src/classification/box_tag.h src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h)
target_link_libraries(pipeline_times ${catkin_LIBRARIES})

add_executable(hd_demo src/hd_demo.cpp src/segment_depth/segment.cpp src/utilities/file_management.h src/utilities/frame.h  src/classification/descriptor.h  src/classification/box_tag.h src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h)
target_link_libraries(hd_demo ${catkin_LIBRARIES})

add_executable(hd_evaluate src/hd_evaluate.cpp src/segment_depth/segment.cpp src/utilities/file_management.h src/utilities/frame.h  src/classification/descriptor.h  src/classification/box_tag.h src/segment_depth/segment-image.h src/segment_depth/segment-graph.h src/segment_depth/disjoint-set.h src/segment_depth/pnmfile.h src/merge_and_filter/merge_and_filter.h src/merge_and_filter/candidate.h)
target_link_libraries(hd_evaluate ${catkin_LIBRARIES})
