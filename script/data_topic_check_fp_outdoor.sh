#! /bin/bash
# gogojjh
cd ~/all_ws
source devel/setup.bash

echo "Check data"
rosrun data_outlier_detection data_topic_check_fp \
  /os_cloud_node/points \
  /stereo/frame_left/image_raw/compressed \
  /stereo/frame_right/image_raw/compressed \
  /stim300/imu/data_raw \
  /fix

