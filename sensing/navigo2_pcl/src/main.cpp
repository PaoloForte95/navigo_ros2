// Copyright (c) 2024  Paolo Forte

// This file is part of pcl_example.


#include <navigo2_pcl/pointcloud_filter.hpp>


int main(int argc, char * argv[])
{
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigo2_pcl::PointCloudFilter>("ouster_filter");
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}