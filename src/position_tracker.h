#ifndef _POSITION_TRACKER_H_
#define _POSITION_TRACKER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <nlink_parser/LinktrackAoaNodeframe0.h>
#include <mavros/mavros.h>
#include <sensor_msgs/Imu.h>
#include <nvcsiapriltag/AprilTagDetectionArray.h>
#include <nvcsiapriltag/AprilTagDetection.h>
#include <string>


class viconobject
{
    ros::Publisher pub;
    ros::Subscriber sub;
    visualization_msgs::Marker marker;
    geometry_msgs::Pose pose;
    tf::TransformBroadcaster tfbr;
    tf::Transform tftf;
public:
    std::string name;
    void callback(const geometry_msgs::TransformStampedConstPtr &msg);
    viconobject(const std::string str, ros::NodeHandle &nh, int freq, int id);
};

class uwbobject
{
    const static double delta_x;
    const static double delta_y;
    const static double delta_z;
    ros::Publisher pub;
    ros::Subscriber sub;
    visualization_msgs::Marker marker;
    tf::TransformBroadcaster tfbr;
    tf::Transform tftf;
    tf::TransformListener tfls;
public:
    std::string name;
    void callback(const nlink_parser::LinktrackAoaNodeframe0ConstPtr &msg);
    uwbobject(const std::string str, ros::NodeHandle &nh, int freq, int id);
};

class apriltagobject
{
    ros::Publisher pub;
    ros::Subscriber sub;
    visualization_msgs::Marker marker;
    tf::TransformBroadcaster tfbr;
    tf::Transform tftf;
    tf::TransformListener tfls;
    std::vector<double>tag_size;
    std::string tfbase;
public:
    std::string name;
    void callback(const nvcsiapriltag::AprilTagDetectionArrayConstPtr &msg);
    apriltagobject(const std::string str, ros::NodeHandle &nh, int freq,  int id, std::vector<double> _tag_size, const std::string _tfbase);
};

#endif // _POSITION_TRACKER_H_