#! /bin/bash
# gogojjh
cd ~/all_ws
source devel/setup.bash

echo "Check data of Big PC"
rosrun data_outlier_detection data_topic_check_big_pc \
  /stereo/frame_left/image_raw/compressed \
  /stereo/frame_right/image_raw/compressed \
  /stim300/imu/data_raw 
