#include "position_tracker.h"
// #include <opencv2/imgproc.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>

// #include <opencv2/core/cuda.hpp>


viconobject::viconobject(const std::string str, ros::NodeHandle &nh, int freq, int id)
{
    name=str;
    uint32_t shape = visualization_msgs::Marker::SPHERE;
    marker.ns = "basic_shapes";
    marker.header.frame_id = "/world";
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    sub=nh.subscribe(name, freq, &viconobject::callback, this);
    pub=nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
}    

void viconobject::callback(const geometry_msgs::TransformStampedConstPtr &msg)
{
    marker.header.stamp=msg->header.stamp;
    marker.header.seq=msg->header.seq;
    const geometry_msgs::Vector3 &vpos=msg->transform.translation;
    const geometry_msgs::Quaternion &qpos=msg->transform.rotation;
    tftf.setOrigin(tf::Vector3(vpos.x, vpos.y, vpos.z));
    tftf.setRotation(tf::Quaternion(qpos.x, qpos.y, qpos.z, qpos.w));
    marker.pose.position.x=vpos.x;
    marker.pose.position.y=vpos.y;
    marker.pose.position.z=vpos.z;
    marker.pose.orientation=msg->transform.rotation;
    pub.publish(marker);
    tfbr.sendTransform(tf::StampedTransform(tftf, ros::Time::now(), "world", name));
}


const double uwbobject::delta_x=0.0;
const double uwbobject::delta_y=0.0;
const double uwbobject::delta_z=0.0;

uwbobject::uwbobject(const std::string str, ros::NodeHandle &nh, int freq, int id)
{
    name=str;
    marker.ns = "basic_shapes";
    marker.header.frame_id = "/world";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    sub=nh.subscribe(name, freq, &uwbobject::callback, this);
    pub=nh.advertise<visualization_msgs::Marker>("visuallization_marker", 200);
}


double zcal(const float &disx, const double &angx, const double &disy, const double &angy, const double &uwboffset)
{
    double Angx=-M_PI*angx/180.0;
    double Angy=-M_PI*angy/180.0;
    double rx=disx*cos(Angx);
    double offsetx=disx*sin(Angx)+uwboffset;
    double ry=disy*cos(Angy);
    double offsety=disy*sin(Angy)+uwboffset;
    double z1=sqrt(rx*rx-offsety*offsety);
    double z2=sqrt(ry*ry-offsetx*offsetx);
    return (z1+z2)/2.0;
}

void uwbobject::callback(const nlink_parser::LinktrackAoaNodeframe0ConstPtr &msg)
{
    marker.header.stamp=ros::Time::now();
    double output_x=-delta_z-zcal(msg->nodes[0].dis, msg->nodes[0].angle, msg->nodes[1].dis, msg->nodes[1].angle, 0.25);
    double output_y= delta_x+msg->nodes[1].dis*sin(-msg->nodes[1].angle*M_PI/180.0);
    double output_z= delta_y+msg->nodes[0].dis*sin(-msg->nodes[0].angle*M_PI/180.0);
    tftf.setOrigin(tf::Vector3(output_x,output_y,output_z));
    tf::Quaternion tfqutemp;
    tfqutemp.setRPY(0,0,0);
    tftf.setRotation(tfqutemp);
    tfbr.sendTransform(tf::StampedTransform(tftf, ros::Time::now(), "/vicon/uwb_anchor/uwb_anchor", name));
    tf::StampedTransform tftemp;
    try
    {
        tfls.lookupTransform(name, "world", ros::Time::now(), tftemp);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("TF Error %f %f %f %f",msg->nodes[0].dis, msg->nodes[0].angle, msg->nodes[1].dis, msg->nodes[1].angle);
    }    
    tf::Quaternion qtemp=tftemp.getRotation();
    geometry_msgs::Quaternion &gq=marker.pose.orientation;
    gq.x=qtemp.getX();
    gq.y=qtemp.getY();
    gq.z=qtemp.getZ();
    gq.w=qtemp.getW();
    marker.pose.position.x=tftemp.getOrigin().getX();
    marker.pose.position.y=tftemp.getOrigin().getY();
    marker.pose.position.z=tftemp.getOrigin().getZ();
    pub.publish(marker);
}

apriltagobject::apriltagobject(const std::string str, ros::NodeHandle &nh, int freq,  int id, std::vector<double> _tag_size, const std::string _tfbase="/vicon/uwb_son/uwb_son")
{
    name=str;
    marker.ns = "basic_shapes";
    marker.header.frame_id = "/world";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    tag_size=_tag_size;
    tfbase=_tfbase;
    sub=nh.subscribe(name, freq, &apriltagobject::callback, this);
    pub=nh.advertise<visualization_msgs::Marker>("apriltag", 100);
}

void apriltagobject::callback(const nvcsiapriltag::AprilTagDetectionArrayConstPtr &msg)
{
    const auto &cnt=msg->detections.size();
    for(auto i=0;i<cnt;++i)
    {
        if(i)   continue;
        const nvcsiapriltag::AprilTagDetection &det=msg->detections[i];
        const geometry_msgs::Quaternion &gqtem=det.pose.pose.pose.orientation;
        double apriltag_k=10.0;
        tftf.setRotation(tf::Quaternion(gqtem.x, gqtem.y, gqtem.z,gqtem.w));
        // tftf.inverse();
        if(det.id>=0&&det.id<tag_size.size())
        {
            apriltag_k=tag_size[det.id];
        }
        tftf.setOrigin(tf::Vector3(det.translation[0], det.translation[1], det.translation[2])*apriltag_k);
        // if(det.id==0)
        //     tfbr.sendTransform(tf::StampedTransform(tftf, ros::Time::now(), tfbase, name+std::to_string(i)));
        // tftf=tftf.inverse();
        if(det.id==0)
        {
            tfbr.sendTransform(tf::StampedTransform(tftf.inverse(), ros::Time::now(), tfbase, name+std::to_string(i)));
            ROS_INFO("Apriltag x = %2.3f\ty = %2.3f\tz = %2.3f", det.translation[0], det.translation[1], det.translation[2]);
            std::vector<cv::Point2f>points;
            std::vector<cv::Point3f>inputP{cv::Point3f(-1,1,0), cv::Point3f(1,1,0), cv::Point3f(1,-1,0), cv::Point3f(-1,-1,0)};
            for(int i=0;i<4;++i)
            {
                ROS_INFO("Point: x = %f\ty = %f", det.corners[i<<1], det.corners[(i<<1)|1]);
                points.push_back(cv::Point2f(det.corners[i<<1],det.corners[(i<<1)|1]));
            }
            // cv::Mat cammat=(cv::Mat_<double>(3,3)<<8.6640902038184322e+02,0.0,6.3950000000000000e+02,0,8.6640902038184322e+02,3.5950000000000000e+02,0.0,0.0,1.0);
            // cv::Mat cofmat=(cv::Mat_<double>(1,5)<<0.0, 0.0, 0.0, 0.0, 0.0);
            // // cv::Mat cofmat;
            // cv::Mat rvec1, tvec1;
            // cv::solvePnP(inputP,points, cammat, cofmat, rvec1, tvec1, false,cv::SOLVEPNP_IPPE_SQUARE);
            // std::cout<<rvec1<<std::endl;
            // std::cout<<tvec1<<std::endl;
        }
    }
}
