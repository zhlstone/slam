/*
 * File Name double_laser
 * Author    Zhoulei
 * Data      Mon Mar  CST 2018
 * Description This programm used two Lidar to recover the
 * building.One laser is used to estimate the pose of the robot,
 * while the other to recover the building structure.
 */

#include<ros/ros.h>
#include <ros/console.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include<laser_geometry/laser_geometry.h>

#include<tf/tf.h>
#include<tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>

#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/registration/transforms.h>
#include<pcl/io/pcd_io.h>

#include<iostream>
#include <boost/thread.hpp>
#include"Eigen/Eigen"
using namespace Eigen;
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_out(new  pcl::PointCloud<pcl::PointXYZ>);
class double_laser
{
public:
    double_laser();
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
private:
    int laser_count;
    int last_laser;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string laser_frame_;
    std::string laser1_frame_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    tf::TransformListener tf_;
     
    ros::NodeHandle node_;
    ros::Publisher pcl_pub;    
};

double_laser::double_laser():
 laser_count(0), last_laser(0)
{
 // tf_ = new TransformListener();
//     Retrieve parameters
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("map_frame", map_frame_))
     map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if(!private_nh_.getParam("base_frame", base_frame_))
	  base_frame_ = "base_link";
  if(!private_nh_.getParam("laser_frame", laser_frame_))
     laser_frame_ = "hokuyo_laser";
  
   scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "hokuyo_scan", 5);
   scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, laser_frame_, 5);
   scan_filter_->registerCallback(boost::bind(&double_laser::LaserCallback, this,_1));
   ROS_INFO("Publish is coming!");
   pcl_pub =node_.advertise<sensor_msgs::PointCloud2>("laser_assemble",5);
}

 void double_laser::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    laser_count++;
    // downsampling
    if(laser_count<(last_laser+10))
        return;
    last_laser=laser_count;
   // tf::TransformListener  listener_;
    tf::StampedTransform map_to_laser;
   try
   {
       
       //listener_.waitForTransform(map_frame_,laser_frame_, ros::Time(0),ros::Duration(1));
      tf_.lookupTransform(map_frame_,laser_frame_,ros::Time(0), map_to_laser);
       ROS_INFO("tf is coming!");
   }
    catch(tf::TransformException& e)
    {
    ROS_WARN("Failed to compute laser, aborting initialization (%s)", 
             e.what());
     return;
    }   
 
 //  pointcloud types transform
    sensor_msgs::PointCloud ros_cloud_in;
    sensor_msgs::PointCloud2 ros_cloud2_in;
    sensor_msgs::PointCloud2 output;
     pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new  pcl::PointCloud<pcl::PointXYZ>);
    laser_geometry::LaserProjection projector_;
    
    projector_.transformLaserScanToPointCloud (laser_frame_,*scan,ros_cloud_in,tf_);
    
    sensor_msgs::convertPointCloudToPointCloud2(ros_cloud_in, ros_cloud2_in);
    pcl::fromROSMsg(ros_cloud2_in,*cloud_in);
    
//     convert tf::transform  to eigen::Matrix
     Eigen::Affine3d Ti=Eigen::Affine3d::Identity();
     tf::transformTFToEigen (map_to_laser,Ti);
     ROS_INFO("laser is coming!");
     pcl::transformPointCloud(*cloud_in,*transformed_cloud,Ti);
   *cloud_out+= *transformed_cloud;
//      pcl::transformPointCloud(*cloud_in,*cloud_out,Ti);
//    *cloud_out+= *cloud_in;
    pcl::toROSMsg(*cloud_out,output);
    output.header.frame_id="map";
    pcl_pub.publish(output);
}
       
int main(int argc, char** argv)
{
   ros::init(argc, argv, "laser_assembler");
   double_laser  laser;    
   ros::spin();
   return 0;
}    