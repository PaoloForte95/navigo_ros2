#include <rclcpp/rclcpp.hpp>
#include <orunav_msgs/msg/fork_command.hpp>
#include <boost/program_options.hpp>
#include <orunav2_generic/utils.hpp>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  po::options_description desc("Allowed options");
  
  int robot_id;
  int command;
  desc.add_options()
      ("help", "produce help message")
      ("robot_id", po::value<int>(&robot_id)->default_value(1), "robot ID (if < 0 the fork command topic will not have any /robotX added")
    ("command", po::value<int>(&command)->default_value(0), "0: move the forks up, 1: move the forks down, 2: activate support legs - move forks all the way down");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  
  if (vm.count("help")) {
      cout << desc << "\n";
      
      return 1;
  }
  
  po::notify(vm);    
  

  auto nh_ = rclcpp::Node::make_shared("fork_control_client");
  std::string fork_command_topic; 
  if (robot_id >=0)
      fork_command_topic = orunav2_generic::getRobotTopicName(robot_id, "/fork/command");
  else
      fork_command_topic = std::string("/fork/command");
  
  
  auto forkcmd_pub = nh_->create_publisher<orunav_msgs::msg::ForkCommand>(fork_command_topic, 1);
  RCLCPP_INFO(nh_->get_logger(), "Sending ForkCommand to : %s\n", fork_command_topic.c_str());

  rclcpp::Rate r(1);
  
  orunav_msgs::msg::ForkCommand cmd;
  cmd.robot_id = robot_id;
  switch (command) {
    case 0:
      RCLCPP_INFO(nh_->get_logger(), "%s", "moving forks UP");
      cmd.state.position_z = 0.1;
      break;
    case 1:
      cmd.state.position_z = 0.0;
      RCLCPP_INFO(nh_->get_logger(), "%s", "moving forks DOWN");
      break;
    case 2:
      cmd.state.position_z = -0.1;
      RCLCPP_INFO(nh_->get_logger(), "%s", "activating support legs - moving all the way DOWN");
      break;
    default:
      RCLCPP_ERROR(nh_->get_logger(), "%s","Not a valid command param...");
      exit(-1);
      break;
  }

  while (rclcpp::ok()) {
   RCLCPP_INFO(nh_->get_logger(),"Sending fork command : %d, position z : %f", command, cmd.state.position_z);
      forkcmd_pub->publish(cmd);
      rclcpp::spin_some(nh_);
      r.sleep();
  }
  return 0;
}

