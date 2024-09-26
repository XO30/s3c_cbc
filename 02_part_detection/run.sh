#!/bin/bash

# Source ROS 2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Navigate to the workspace
cd /root/colcon_ws

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Set ROS_DOMAIN_ID if needed
export ROS_DOMAIN_ID=0

# Start the realsense2 camera node
ros2 launch realsense2_camera rs_launch.py &

# Run the part_detection_node from the part_detection_pkg package.
# This command includes various parameters specified using the -p option.
ros2 run part_detection_pkg part_detection_node \
  --ros-args \
  -p depth_image_topic:='/camera/camera/depth/image_rect_raw' \
  -p depth_info_topic:='/camera/camera/depth/camera_info' \
  -p rgb_image_topic:='/camera/camera/color/image_raw' \
  -p rgb_info_topic:='/camera/camera/color/camera_info' \
  -p depth_plane_threshold:=15 \
  -p depth_dilation:='[20, 20]' \
  -p topics:=true \
  -p services:=true \
  -p depth_area_threshold:=5000 \
  -p num_stack:=5 \
  -p detetion_distance_threshold:=325 \
  -p rgb_blob_max_pairing_distance:=50 \
  -p rgb_circle_max_pairing_distance:=50 \
  -p rgb_min_keypoint_size:=27 \
  -p rgb_blob_min_threshold:=10 \
  -p rgb_blob_max_threshold:=150 \
  -p rgb_blob_threshold_step:=10 \
  -p rgb_blob_min_area:=150 \
  -p rgb_blob_min_circularity:=0.6 \
  -p rgb_circle1_min_radius:=35 \
  -p rgb_circle1_max_radius:=60 \
  -p rgb_circle2_min_radius:=70 \
  -p rgb_circle2_max_radius:=110 \
  -p rgb_medianBlurKSize:=5 \
  -p rgb_cannyThreshold1:=20 \
  -p rgb_cannyThreshold2:=40 \
  -p rgb_houghDP:=1 \
  -p rgb_houghMinDist:=100 \
  -p rgb_houghParam1:=100 \
  -p rgb_houghParam2:=30 \
  -p depth2rgbx:=1.36 \
  -p depth2rgby:=0.98 \
  -p offsetx:=0.8 \
  -p offsety:=0.0 \
  -p areas:='"[ [1, 8200, 50], [2, 22000, 50] ]"' \
  -p rgb_gaussianBlurKSize:='[5, 5]' \
  -p home:='[400.0, -280.0, 625.0, 0.0, 3.543, 0.0]' \
  -p height_1:=475 \
  -p end_height:=350 \
  -p transformation:='[37.2, 32.1, 0.0, 0.0, 0.401, 0.0]' &

# Loop to keep the container running
while true; do
    sleep 10
done
