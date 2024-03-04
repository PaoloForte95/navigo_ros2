#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

class SpeedLimitSubscriber : public rclcpp::Node
{
public:
    SpeedLimitSubscriber() : Node("speed_limit_subscriber")
    {
        subscription_ = this->create_subscription<nav2_msgs::msg::SpeedLimit>(
            "speed_limit",
            10,
            std::bind(&SpeedLimitSubscriber::speedLimitCallback, this, std::placeholders::_1));

        desired_velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "desired_velocity_topic",
            10,
            std::bind(&SpeedLimitSubscriber::desiredVelocityCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void speedLimitCallback(const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
    {
        // Assuming that the speed limit message contains only linear velocity
        geometry_msgs::msg::Twist cmd_vel;
        speed_limit_ = msg->speed_limit;
        percentage_ = msg->percentage;
        cmd_vel.angular.z = 0.0; // Assuming no angular velocity
      

    }

    void desiredVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Apply speed limit to desired velocity
        geometry_msgs::msg::Twist filtered_cmd_vel;
        filtered_cmd_vel.linear.x =  msg->linear.x;
        filtered_cmd_vel.linear.y =  msg->linear.y;
        if(speed_limit_ > 0){
            if (percentage_) {
            // Speed limit is expressed in % from maximum speed of robot
                max_speed_=
                filtered_cmd_vel.linear.x =  msg->linear.x * speed_limit_ / 100.0;
                filtered_cmd_vel.linear.y =  msg->linear.y * speed_limit_ / 100.0;
            } else {
                //Speed limit is expressed in absolute value
                filtered_cmd_vel.linear.x =  speed_limit_;
                filtered_cmd_vel.linear.y =  speed_limit_;
            }
        }

        filtered_cmd_vel.linear.z = 0;
        filtered_cmd_vel.angular.x = 0;
        filtered_cmd_vel.angular.y = 0;
        filtered_cmd_vel.angular.z = msg->angular.z;
        cmd_vel_publisher_->publish(filtered_cmd_vel);
    }

    rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_velocity_subscription_;
     double max_speed_ = 1.0;
     double speed_limit_ = 1.0;
     bool percentage_;

    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeedLimitSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
