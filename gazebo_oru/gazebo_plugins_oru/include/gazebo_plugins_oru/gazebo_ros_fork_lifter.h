#ifndef FORK_LIFTER_PLUGIN_HH
#define FORK_LIFTER_PLUGIN_HH

#include <map>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Custom Callback Queue
#include <rclcpp/callback_group.hpp>
#include <rclcpp/publisher_options.hpp>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

class Joint;
class Entity;


class GazeboRosForkLifter : public ModelPlugin {

public:
    GazeboRosForkLifter();
    ~GazeboRosForkLifter();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();
    virtual void FiniChild();

private:
    GazeboRosPtr gazebo_ros_;
    physics::ModelPtr parent;
    //void publishOdometry(double step_time);
    void publishTF(); /// publishes the wheel tf's
    void publishJointState();
    void motorController(double target_fork_height, double dt);

    event::ConnectionPtr update_connection_;

    physics::JointPtr joint_fork_;

    double fork_torque_;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string robot_base_frame_;

    // ROS2 STUFF
    ros::Subscriber cmd_fork_subscriber_;
    boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;

    boost::mutex lock;

    // Custom Callback Queue
    rclcpp::CallbackGroup group_
    boost::thread callback_queue_thread_;
    void QueueThread();

    // DiffDrive stuff
    void cmdForkCallback(const geometry_msgs::Point::ConstPtr& cmd_msg);

    /// updates the relative robot pose based on the wheel encoders
    void UpdateForkEncoder();

    double fork_height_cmd_;

    bool alive_;
    double fork_height_encoder_;
    common::Time last_encoder_update_;
    double fork_height_origin_;
    common::PID  joint_pid_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_actuator_update_;

    // Flags
    bool publishTF_;
    bool publishJointState_;

    bool use_velocity_control_;
    double max_velocity_;
};

}

#endif

