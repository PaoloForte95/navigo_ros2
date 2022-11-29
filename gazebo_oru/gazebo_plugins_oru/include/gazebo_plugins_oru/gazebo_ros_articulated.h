#ifndef ARTICULATED_PLUGIN_HH
#define ARTICULATED_PLUGIN_HH

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

#include <articulated_geometry/odometry.h>

namespace gazebo {

class Joint;
class Entity;


class GazeboRosArticulated : public ModelPlugin {

    class ArticulatedCmd {
    public:
        ArticulatedCmd():speed(0), angle(0) {};
        double speed;
        double angle;
    };

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
public:
    GazeboRosArticulated();
    ~GazeboRosArticulated();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();
    virtual void FiniChild();

private:
    GazeboRosPtr gazebo_ros_;
    physics::ModelPtr parent;
    void publishOdometry(double step_time);
    void publishWheelTF(); /// publishes the wheel tf's
    void publishWheelJointState();
    void motorController(double target_speed, double target_steering_speed, double dt);

    event::ConnectionPtr update_connection_;

    physics::JointPtr joint_articulated_steer_; // The joint doing the steering.
    
    int nb_axis_rear_; // For the rear part you might have more axis, e.g. a boogie axis

    physics::JointPtr joint_wheel_fl_drive_;
    physics::JointPtr joint_wheel_fr_drive_;
    physics::JointPtr joint_wheel_bl_drive_;
    physics::JointPtr joint_wheel_br_drive_;
    //physics::JointPtr joint_wheel_bl2_drive_;
    //physics::JointPtr joint_wheel_br2_drive_;

    double length_front_axis_;
    double length_rear_axis_;
    double wheel_diameter_front_;
    double wheel_diameter_rear_;
    double min_steering_angle_;
    double max_steering_angle_;

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

    // ROS STUFF
    ros::Publisher odometry_publisher_;
    ros::Subscriber cmd_vel_subscriber_;
    boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;
    nav_msgs::Odometry odom_;

    boost::mutex lock;


    // Custom Callback Queue
    rclcpp::CallbackGroup group_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    // DiffDrive stuff
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    /// updates the relative robot pose based on the wheel encoders
    void UpdateOdometryEncoder();

    ArticulatedCmd cmd_;
    bool alive_;
    geometry_msgs::Pose2D pose_encoder_;
    common::Time last_odom_update_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_actuator_update_;

    //ecl::mapping::articulated_geometry::ArticulatedOdometry odometry_model_;
    wheel_loader_geometry::AckermannModelFromSteerDrive wheel_model_;

    // Flags
    bool publishWheelTF_;
    bool publishWheelJointState_;

};

}

#endif

