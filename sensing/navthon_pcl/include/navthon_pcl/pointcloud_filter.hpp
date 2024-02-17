// Copyright (c) 2024 Paolo Forte


#ifndef NAVTHON_PCL__POINTCLOUD_FILTER_NODE_HPP_
#define NAVTHON_PCL__POINTCLOUD_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
/**
 * @class PointCloudFilter
 * @brief Receives Pointcloud2 message from lidar sensor and filter its points with an optional pcl filter.
 * 
 */

namespace navthon_pcl
{

class PointCloudFilter : public rclcpp_lifecycle::LifecycleNode
{
  public:
    
    /**
     * @brief A constructor for PointCloudFilter class
     * @param options Additional options to control creation of the node.
     */
    explicit PointCloudFilter(  const std::string & node_name,
  const std::string & ns = "",
  const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief A destructor for PointCloudFilter class
     */
    ~PointCloudFilter() {};


        CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Activate member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Deactivate member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Reset member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Called when in shutdown state
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  protected:
    /**
     * @brief Use a no filter of pcl library
     * @param msg Pointcloud2 message receveived from the ros2 node
     * @return -
     * @details Omit pointcloud filtering in this example
     */
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

 
    // ROS2 subscriber and related topic name
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::string param_topic_pointcloud_in;
    
    // ROS2 publisher and related topic name 
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string topic_in_, topic_out_;
    int min_neigh_;
    double radius_;
    
};


}
#endif 