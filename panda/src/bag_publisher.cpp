#include "ros/ros.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_publisher");
  ros::NodeHandle nh;

  ros::Publisher point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);
  ros::Rate loop_rate(0.1);
  
  rosbag::Bag bagfile; // Variable holding the rosbag containing point cloud data.
  std::string path = ros::package::getPath("panda");
  path += "/bags/pointcloud.bag";
  bagfile.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("/camera/depth_registered/points");

  rosbag::View bagtopics_iter(bagfile, rosbag::TopicQuery(topics));// Iterator for topics in bag.

  for (auto const msg : bagtopics_iter)
  {
    sensor_msgs::PointCloud2::Ptr point_cloud_ptr = msg.instantiate<sensor_msgs::PointCloud2>();
    if (point_cloud_ptr == NULL)
    {
      std::cout << "error" << std::endl;
      break;
    }

    while (ros::ok())
    {
      point_cloud_ptr->header.stamp = ros::Time::now();
      point_cloud_publisher.publish(*point_cloud_ptr);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  bagfile.close();
  return 0;
}