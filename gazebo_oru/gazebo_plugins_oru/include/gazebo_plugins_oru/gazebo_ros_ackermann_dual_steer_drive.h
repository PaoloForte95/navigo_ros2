#ifndef ACKERMANN_DUAL_STEER_DRIVE_PLUGIN_HH
#define ACKERMANN_DUAL_STEER_DRIVE_PLUGIN_HH

#include <map>
#include <memory>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

//GAZEBO
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>

// Custom Callback Queue
#include <rclcpp/callback_group.hpp>
#include <rclcpp/publisher_options.hpp>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>


#include <orunav2_generic/ackermann.h>

namespace gazebo {

class Joint;
class Entity;



class GazeboRosAckermannDualSteerDrive : public ModelPlugin {

    class DualAckermannDriveCmd {
    public:
      DualAckermannDriveCmd():speed(0), angle1(0), angle2(0) {};
      double speed;
      double angle1; // front steering angle
      double angle2; // rear steering angl
    };

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
public:
    GazeboRosAckermannDualSteerDrive();
    ~GazeboRosAckermannDualSteerDrive();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();
    virtual void FiniChild();
    

private:
    gazebo_ros::Node::SharedPtr gazebo_ros_;
    physics::ModelPtr parent;
    void publishOdometry(double step_time);
    void publishWheelTF(); /// publishes the wheel tf's
    void publishWheelJointState();
    void motorController(double target_speed, double target_front_steering_speed, double target_rear_steering_speed, double dt);

    event::ConnectionPtr update_connection_;

    
    std::string joint_name;
    physics::JointPtr joint_front_steer_; // The virtual centered front steering.
    physics::JointPtr joint_rear_steer_;  // The virtual centered rear steering.
    physics::JointPtr joint_wheel_fl_steer_;
    physics::JointPtr joint_wheel_fr_steer_;
    physics::JointPtr joint_wheel_bl_steer_;
    physics::JointPtr joint_wheel_br_steer_;

    physics::JointPtr joint_wheel_fl_drive_;
    physics::JointPtr joint_wheel_fr_drive_;
    physics::JointPtr joint_wheel_bl_drive_;
    physics::JointPtr joint_wheel_br_drive_;

    double wheel_diameter_;
    double wheel_acceleration_;
    double wheel_deceleration_;
    double wheel_speed_tolerance_;
    double steering_angle_tolerance_;
    double steering_speed_;
    double separation_encoder_wheel_;

    OdomSource odom_source_;
    double steer_torque_;
    double drive_torque_;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string odometry_topic_;
    std::string odometry_frame_;
    std::string robot_base_frame_;

    // ROS2 STUFF
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    sensor_msgs::msg::JointState joint_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    nav_msgs::msg::Odometry odom_;

    boost::mutex lock;


    // Custom Callback Queue
    rclcpp::CallbackGroup group_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    // DiffDrive stuff
    void cmdVelCallback(const geometry_msgs::msg::Twist::ConstPtr& cmd_msg);

    /// updates the relative robot pose based on the wheel encoders
    void UpdateOdometryEncoder();

    DualAckermannDriveCmd cmd_;
    bool alive_;
    geometry_msgs::msg::Pose2D pose_encoder_;
    common::Time last_odom_update_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_actuator_update_;

    orunav2_generic::AckermannModelFromDualSteerDrive ackermann_model_;
  
    // Flags
    bool publishWheelTF_;
    bool publishWheelJointState_;

};

}

#endif

