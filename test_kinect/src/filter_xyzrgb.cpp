#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"

#include <iterator>


void on_pcl(const sensor_msgs::PointCloud2ConstPtr& points_in_msg,
	    ros::Publisher& pcl_pub) {
  // Convert to PCL data type
  pcl::PointCloud<pcl::PointXYZRGB> points_in;
  pcl::fromROSMsg (*points_in_msg, points_in);


  pcl::PointCloud<pcl::PointXYZRGB> points_out;
  auto out = std::back_inserter(points_out.points);
  std::copy_if(points_in.begin(), points_in.end(), 
  	       out, 
  	       [](const pcl::PointXYZRGB& p) -> bool {return p.z < 1.5;});


  // We publish the result.
  pcl::PCLPointCloud2      points_2;
  pcl::toPCLPointCloud2(points_out, points_2);

  sensor_msgs::PointCloud2 points_out_msg;
  pcl_conversions::fromPCL(points_2, points_out_msg);

  points_out_msg.header.stamp = ros::Time::now();
  points_out_msg.header.frame_id = points_in_msg->header.frame_id;
  pcl_pub.publish(points_out_msg);

}


int main(int argc, char* argv[]) {
  // Initialize ROS
  ros::init (argc, argv, "filter_xyzrgb");

  ros::NodeHandle nh;

  // Create a ROS publisher for the output point cloud
  ros::Publisher pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("output_pcl", 1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input_pcl", 1, boost::bind(on_pcl, _1, boost::ref(pcl_pub)));

  // Spin
  ros::spin ();
}
