// Copyright (c) 2024 Paolo Forte


#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "navthon_pcl/pointcloud_filter.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;


namespace navthon_pcl
{
PointCloudFilter::PointCloudFilter(const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
{
      
  declare_parameter<std::string>("topic_in","bf_lidar/point_cloud_out");
  declare_parameter<std::string>("topic_out", "bf_lidar/point_cloud_pcl_example");
  declare_parameter<int>("min_neigh", 5);
  declare_parameter<double>("radius", 1.0);
  
  topic_in_ = get_parameter("topic_in").as_string();
  topic_out_ = get_parameter("topic_out").as_string();

  radius_ = get_parameter("radius").as_double();
  min_neigh_ = get_parameter("min_neigh").as_int();
  
  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       pcl_example\n"
  "Subscribes: Pointcloud2 message: %s\n"
  "Publishes:  Pointcloud2 message: %s \n"
  "Details:    Uging statistical Outlier Removal.\n"
  "Running...", topic_in_.c_str(),topic_out_.c_str());

}

CallbackReturn
PointCloudFilter::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring PointCloudFilter node");
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_out_,2);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_in_, 10, std::bind(&PointCloudFilter::topic_callback, this, _1));

}

CallbackReturn
PointCloudFilter::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating PointCloudFilter node");

  return CallbackReturn::SUCCESS;
}


CallbackReturn
PointCloudFilter::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating PointCloudFilter node ");
  return CallbackReturn::SUCCESS;
}


CallbackReturn
PointCloudFilter::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up PointCloudFilter node");
  publisher_.reset();
  subscription_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
PointCloudFilter::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down PointCloudFilter node");
  return CallbackReturn::SUCCESS;
}

void PointCloudFilter::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  unsigned int num_points = msg->width;
  RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", num_points);
    

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // ROS2 Pointcloud2 to PCL Pointcloud2
  
  pcl_conversions::toPCL(*msg,*cloud);    
                       

    // Create the filtering object

  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;

  sor.setInputCloud(cloud);

  sor.setMeanK(min_neigh_);

  sor.setStddevMulThresh(radius_);

  sor.filter(*cloud_filtered);

  // PCL message to ROS2 message 

  sensor_msgs::msg::PointCloud2 cloud_out;

  pcl_conversions::fromPCL(*cloud_filtered,cloud_out);  

  unsigned int num_points_out = cloud_out.width;
  RCLCPP_INFO(this->get_logger(), "The number of points in the pcl_example_out pointcloud is %i", num_points_out);

  cloud_out.header.frame_id = msg->header.frame_id;
  cloud_out.header.stamp = msg->header.stamp;

  // Publish to ROS2 network
  publisher_->publish(cloud_out);
}

}