#!/bin/sh
rosrun topic_tools throttle messages /camera/color/image_raw 2.0 /camera/color/image_raw_t &
rosrun topic_tools throttle messages /camera/depth/image_rect_raw 2.0 /camera/depth/image_rect_raw_t &
rosrun topic_tools throttle messages /camera/aligned_depth_to_color/image_raw 2.0 /camera/aligned_depth_to_color/image_raw_t &
rosrun topic_tools throttle messages /dynamic_map/inflated_voxel_map 4.0 /dynamic_map/inflated_voxel_map_t