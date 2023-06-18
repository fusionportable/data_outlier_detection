#! /bin/bash
# gogojjh
cd ~/all_ws
source devel/setup.bash

echo "Check data of Small PC"
rosrun data_outlier_detection data_topic_check_small_pc \
  /os_cloud_node/points \
  /stereo/davis_left/image_raw/compressed \
  /stereo/davis_right/image_raw/compressed \
  /3dm_ins/imu/data_raw \
  /mini_hercules/encoder 
