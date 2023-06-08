// Usage: rosrun data_outlier_detection data_topic_check_fp \
  /os_cloud_node/points \
  /stereo/frame_left/image_raw/compressed \
  /stereo/frame_right/image_raw/compressed \
  /stim300/imu/data_raw

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>

// ******************************** define global variables
const double check_time_interval = 2.0;  // s
double latest_lidar_time = 0.0;
double latest_frame_cam00_time = 0.0;
double latest_frame_cam01_time = 0.0;
double latest_imu_time = 0.0;
double latest_gps_time = 0.0;
double latest_gps_3dm_time = 0.0;
double latest_event_cam00_time = 0.0;
double latest_event_cam01_time = 0.0;
double latest_encoder_time = 0.0;
// ******************************

void LiDARCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  latest_lidar_time = msg->header.stamp.toSec();
}

void FrameCam00Callback(const sensor_msgs::CompressedImageConstPtr& msg) {
  latest_frame_cam00_time = msg->header.stamp.toSec();
}

void FrameCam01Callback(const sensor_msgs::CompressedImageConstPtr& msg) {
  latest_frame_cam01_time = msg->header.stamp.toSec();
}

void EventCam00Callback(const sensor_msgs::CompressedImageConstPtr& msg) {
  latest_event_cam00_time = msg->header.stamp.toSec();
}

void EventCam01Callback(const sensor_msgs::CompressedImageConstPtr& msg) {
  latest_event_cam01_time = msg->header.stamp.toSec();
}

void IMUCallback(const sensor_msgs::ImuConstPtr& msg) {
  latest_imu_time = msg->header.stamp.toSec();
}

void GPSCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
  latest_gps_time = msg->header.stamp.toSec();
}

void GPS3DMCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
  latest_gps_3dm_time = msg->header.stamp.toSec();
}

void EncoderCallback(const sensor_msgs::JointStateConstPtr& msg) {
  latest_encoder_time = msg->header.stamp.toSec();
}

void CheckData(const double& curr_time, const double sensor_time,
               const std::string info) {
  if (sensor_time >= 0 &&
      std::abs(curr_time - sensor_time) > check_time_interval) {
    std::cout << "Missing " << info << " data ! (" << std::fixed
              << std::setprecision(5) << sensor_time << " " << curr_time << " "
              << std::abs(curr_time - sensor_time) << "s)" << std::endl;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_topic_check_small_pc");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  if (argc == 6) {
    std::string topic1(argv[1]);
    std::string topic2(argv[2]);
    std::string topic3(argv[3]);
    std::string topic4(argv[4]);
    std::string topic5(argv[5]);
    std::cout << "Topic lists: " << std::endl;
    std::cout << "--- lidar topic: " << topic1 << std::endl;
    std::cout << "--- event_cam00 topic: " << topic2 << std::endl;
    std::cout << "--- event_cam01 topic: " << topic3 << std::endl;
    std::cout << "--- 3dm imu topic: " << topic4 << std::endl;
    std::cout << "--- encoder topic: " << topic5 << std::endl;

    ros::Subscriber lidar_sub =
        nh.subscribe<sensor_msgs::PointCloud2>(topic1, 5, LiDARCallback);
    ros::Subscriber event_cam00_sub =
        nh.subscribe<sensor_msgs::CompressedImage>(topic2, 5,
                                                   EventCam00Callback);
    ros::Subscriber event_cam01_sub =
        nh.subscribe<sensor_msgs::CompressedImage>(topic3, 5,
                                                   EventCam01Callback);
    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>(topic4, 5, IMUCallback);
    ros::Subscriber encoder_sub =
        nh.subscribe<sensor_msgs::JointState>(topic4, 5, EncoderCallback);

    double latest_curr_time = ros::Time::now().toSec();

    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      if (std::abs(ros::Time::now().toSec() - latest_curr_time) >
          check_time_interval) {
        latest_curr_time = ros::Time::now().toSec();
        CheckData(latest_lidar_time, latest_curr_time, "lidar");
        CheckData(latest_event_cam00_time, latest_curr_time, "event_cam00");
        CheckData(latest_event_cam01_time, latest_curr_time, "event_cam01");
        CheckData(latest_imu_time, latest_curr_time, "imu");
        CheckData(latest_encoder_time, latest_curr_time, "wheel_encoder");
      }
      loop_rate.sleep();
    }
  }
  return 0;
}
