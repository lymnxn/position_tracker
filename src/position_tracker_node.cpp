#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <vicon_bridge/Markers.h>
#include <vicon_bridge/Marker.h>
#include <visualization_msgs/Marker.h>
#include <nlink_parser/LinktrackAoaNodeframe0.h>
#include <mavros/mavros.h>
#include <sensor_msgs/Imu.h>
#include <nvcsiapriltag/AprilTagDetectionArray.h>
#include <nvcsiapriltag/AprilTagDetection.h>
#include "position_tracker.h"
#include "rclcpp/rclcpp.hpp"

void uwb_callback(const nlink_parser::LinktrackAoaNodeframe0ConstPtr &msg)
{
    ROS_INFO("UWB id=%u\tnode size=%lu", msg->id, msg->nodes.size());
}

void mavros_imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
    ROS_INFO("IMU data %s", msg->header.frame_id.c_str());
}

void mavros_imu_raw_callback(const sensor_msgs::ImuConstPtr &msg)
{
    ROS_INFO("IMU RAW data %s", msg->header.frame_id.c_str());
}

void apriltag_callback(const nvcsiapriltag::AprilTagDetectionArrayConstPtr &msg)
{
    ROS_INFO("Apriltag cnt: %lu", msg->detections.size());
}

class positon_track_node : public rclcpp::Node
{
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr px4_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mocap_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uwb_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /* data */
public:
  positon_track_node(/* args */);
  ~positon_track_node();
};

positon_track_node::positon_track_node(/* args */)
{
}

positon_track_node::~positon_track_node()
{
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracker");
  ros::NodeHandle nh;
  // ros::Subscriber sub_apriltag = nh.subscribe("/apriltag_handle", 1000, apriltag_callback);
  // ros::Subscriber sub_px4_imu = nh.subscribe("/mavros/imu/data", 1000, mavros_imu_callback);
  // ros::Subscriber sub_px4_imu_raw = nh.subscribe("/mavros/imu/data_raw", 1000, mavros_imu_raw_callback);
//   ros::Subscriber sub_uwb = nh.subscribe("/nlink_linktrack_aoa_nodeframe0", 1000, uwb_callback);
  // uwbobject uwb("/nlink_linktrack_aoa_nodeframe0", nh, 1000, 2);
  viconobject tag("/vicon/uav_son/uav_son", nh, 1000, 0);
  viconobject anchor("/vicon/uwb_anchor/uwb_anchor", nh, 1000, 1);
  apriltagobject apriltag("/apriltag_handle", nh, 1000, 3, std::vector<double>{1, 0.05, 0.05, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015, 0.015}, "/vicon/uwb_anchor/uwb_anchor");//"/camera");//the only value in mm
  ros::MultiThreadedSpinner spinner(6);
  spinner.spin();
  return 0;
}