#ifndef FORK_CONTROL_CITI_TRUCK_HH
#define FORK_CONTROL_CITI_TRUCK_HH

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <orunav_msgs/msg/fork_command.hpp>
#include <orunav_msgs/msg/fork_report.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <boost/thread/mutex.hpp>
#include <orunav_msgs/msg/operation_report.hpp>
#include <orunav_msgs/msg/robot_target.hpp>
#include <orunav_msgs/msg/operation.hpp>
#include <rclcpp/timer.hpp>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;
class ForkControlSimNode {

    private:
    std::shared_ptr<rclcpp::Node> nh_ = rclcpp::Node::make_shared("fork_control_sim");

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<orunav_msgs::msg::ForkReport>::SharedPtr forkreport_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr forkcommand_pub_;
    rclcpp::Publisher<orunav_msgs::msg::OperationReport>::SharedPtr operationreport_pub_;

    rclcpp::Subscription<orunav_msgs::msg::ForkCommand>::SharedPtr forkcommand_sub_;
    rclcpp::Subscription<orunav_msgs::msg::RobotTarget>::SharedPtr operation_sub_;


    rclcpp::TimerBase::SharedPtr heartbeat_reports_;
    bool visualize;
    double lift_threshold_;
    double curr_pos_z;
    orunav_msgs::msg::ForkReport::SharedPtr current_report_;
    orunav_msgs::msg::OperationReport::SharedPtr current_operation_report_;
    double fork_max_speed_;
    boost::mutex reports_mutex_;
    
public: 
	ForkControlSimNode (std::shared_ptr<rclcpp::Node> &paramHandle) {
	  
       
	    forkcommand_sub_ = nh_->create_subscription<orunav_msgs::msg::ForkCommand>("fork/command",0,std::bind(&ForkControlSimNode::process_forkcommand, this, _1));
	    operation_sub_ = nh_->create_subscription<orunav_msgs::msg::RobotTarget>("control/operation",0, std::bind(&ForkControlSimNode::process_operation, this, _1));
		double hb_report;
		paramHandle->get_parameter_or("heartbeat",hb_report,0.1);
		paramHandle->get_parameter_or("visualize",visualize,true);
		paramHandle->get_parameter_or("lift_threshold", lift_threshold_, 0.05);
		paramHandle->get_parameter_or("fork_max_speed", fork_max_speed_, 0.04);
		paramHandle->get_parameter_or("curr_pos_z", curr_pos_z, 0.0);
		forkreport_pub_ = nh_->create_publisher<orunav_msgs::msg::ForkReport>("fork/report", 1000);
		forkcommand_pub_ = nh_->create_publisher<geometry_msgs::msg::Point>("cmd_fork", 1000);
		operationreport_pub_ = nh_->create_publisher<orunav_msgs::msg::OperationReport>("operation/report", 1000);


  	    if (visualize)
	    {
                std::cout << "The output is visualized using visualization_markers (in rviz)." << std::endl;
                marker_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1000);
	    }
        std::chrono::duration<double, std::milli> duration(hb_report);
	    heartbeat_reports_   = nh_->create_wall_timer(duration, std::bind(&ForkControlSimNode::publish_reports, this));

            current_report_->status = orunav_msgs::msg::ForkReport::FORK_POSITION_UNKNOWN;
            current_operation_report_->status = orunav_msgs::msg::OperationReport::START_OPERATION_NOT_PERFORMED;
            current_report_->state.position_z = 0.;
        }

	~ForkControlSimNode() {

        }
  
	void publish_reports() {
//        ROS_INFO("[ForkControlSim]: Publishing reports.");
	  reports_mutex_.lock();
      current_report_->stamp = nh_->now();
      auto current_report_message_ = orunav_msgs::msg::ForkReport();
      current_report_message_.robot_id = current_report_->robot_id;
      current_report_message_.stamp = current_report_->stamp;
      current_report_message_.state = current_report_->state;
      current_report_message_.status = current_report_->status;


      auto current_operation_report_message_ = orunav_msgs::msg::OperationReport();
      current_operation_report_message_.robot_id = current_operation_report_ ->robot_id;
      current_operation_report_message_.stamp = current_operation_report_ ->stamp;
      current_operation_report_message_.status = current_operation_report_ ->status;
	  forkreport_pub_->publish(current_report_message_);
	  operationreport_pub_->publish(current_operation_report_message_);
	  reports_mutex_.unlock();
	}

    
    void send_interpolated_fork_cmds(double current, double target, int rate) {
       rclcpp::Rate r(rate);
	if(current != target){ //PF
        
        double time_to_move_forks = fabs(target - current)/this->fork_max_speed_;
        int steps = time_to_move_forks * 10;
        double inc_z = (target - current) / (steps*1.);
        geometry_msgs::msg::Point cmdfork; cmdfork.x = 0; cmdfork.y = 0;
        for (int i = 0; i <= steps; i++) {
            cmdfork.z = current + inc_z * i;
            RCLCPP_INFO(nh_->get_logger(),"[ForkControlSim]: sending new fork command z : %f", cmdfork.z);
            forkcommand_pub_->publish(cmdfork);
            r.sleep();
        } //end for
	} //end if PF

	else{ //PF
	geometry_msgs::msg::Point cmdfork; cmdfork.x = 0; cmdfork.y = 0;
 	cmdfork.z = current;
	RCLCPP_INFO(nh_->get_logger(),"[ForkControlSim]: sending new fork command z PF : %f", cmdfork.z);
            forkcommand_pub_->publish(cmdfork);
	r.sleep();
	}//END else PF

    } //end function



    void set_operation_status(){
        
    	if( current_operation_report_->status == orunav_msgs::msg::OperationReport::START_OPERATION_NOT_PERFORMED){
            
    		current_operation_report_->status = orunav_msgs::msg::OperationReport::START_OPERATION_PERFORMED ;
    	}
    	else if ( current_operation_report_->status == orunav_msgs::msg::OperationReport::GOAL_OPERATION_NOT_PERFORMED ){
    		current_operation_report_->status = orunav_msgs::msg::OperationReport::GOAL_OPERATION_PERFORMED;
    	}
        
    	else if ( current_operation_report_->status == orunav_msgs::msg::OperationReport::START_OPERATION_PERFORMED){
    		current_operation_report_->status = orunav_msgs::msg::OperationReport::GOAL_OPERATION_NOT_PERFORMED;
    	}
    }


    void process_operation(const orunav_msgs::msg::RobotTarget::SharedPtr msg) const{
    	if(msg -> start_op.operation == orunav_msgs::msg::Operation::NO_OPERATION ){
        
    		current_operation_report_->status = orunav_msgs::msg::OperationReport::GOAL_OPERATION_PERFORMED;
            
    	}
    	else {
            
    		current_operation_report_->status = orunav_msgs::msg::OperationReport::START_OPERATION_NOT_PERFORMED;
    	}

    }

    void process_forkcommand(const orunav_msgs::msg::ForkCommand::SharedPtr msg) {

        // NOTE: This is written to be rather citi truck compatible and not to provide more information (than the cititruck) -> only to move up and down + we don't really know where the forks are.
        RCLCPP_INFO(nh_->get_logger(),"[ForkControlSim]: Got ForkCommand z : %f", msg->state.position_z);
        RCLCPP_INFO(nh_->get_logger(),"[ForkControlSim]: Current fork position z : %f", curr_pos_z);


        if (msg->state.position_z > lift_threshold_) {
            // Lift the forks (unless they are already up).
            RCLCPP_INFO(nh_->get_logger(),"%s","[ForkControlSim]: Forks - high");
 		//if (curr_pos_z < msg->state.position_z)
            //if (current_report_.status != current_report_.FORK_POSITION_HIGH) 
		 if (curr_pos_z < msg->state.position_z)
            	{
			 RCLCPP_INFO(nh_->get_logger(), "%s","[ForkControlSim]: Forks - BEB1");
                reports_mutex_.lock();
                current_report_->status = orunav_msgs::msg::ForkReport::FORK_MOVING_UP;
                double current_pos_z = current_report_->state.position_z;
		//curr_pos_z = curr_pos_z + 0.1;
                reports_mutex_.unlock();

                send_interpolated_fork_cmds(current_pos_z, msg->state.position_z, 10);
                
                reports_mutex_.lock();
                current_report_->status = orunav_msgs::msg::ForkReport::FORK_POSITION_HIGH;
                current_report_->state.position_z = msg->state.position_z; // Assume we're up.
                curr_pos_z = msg->state.position_z;
                set_operation_status();
                reports_mutex_.unlock();

            	}

	    else if(curr_pos_z > msg->state.position_z && current_report_->status == orunav_msgs::msg::ForkReport::FORK_POSITION_STACK){
	    		reports_mutex_.lock();
                current_report_->status = orunav_msgs::msg::ForkReport::FORK_MOVING_DOWN;
                double current_pos_z = current_report_->state.position_z;
                reports_mutex_.unlock();

                send_interpolated_fork_cmds(current_pos_z, msg->state.position_z, 10);

                reports_mutex_.lock();
                current_report_->status = orunav_msgs::msg::ForkReport::FORK_POSITION_HIGH;
                current_report_->state.position_z = msg->state.position_z; // Assume we're down.
                curr_pos_z = msg->state.position_z;
                set_operation_status();
                reports_mutex_.unlock();
		}

	    ////////////////////////////
		 //THIS PART IS ADDED



		 /////////////////////////////////////7
	    //else{
		 else{
			 	reports_mutex_.lock();
                current_report_->status = orunav_msgs::msg::ForkReport::FORK_MOVING_UP;
                double current_pos_z = current_report_->state.position_z;
		//curr_pos_z = curr_pos_z + 0.1;
                reports_mutex_.unlock();
                  send_interpolated_fork_cmds(current_pos_z, current_pos_z, 10);
               // double current_pos_z = current_report_.state.position_z;
		//send_interpolated_fork_cmds(current_pos_z, current_pos_z, 10);



                reports_mutex_.lock();
                current_report_->status = orunav_msgs::msg::ForkReport::FORK_POSITION_STACK;
                current_report_->state.position_z = msg->state.position_z; // Assume we're up.
                curr_pos_z = msg->state.position_z;
                set_operation_status();
                reports_mutex_.unlock();

		}
		////////////////////////////
		 ////////////////////////////
        }

        else if (msg->state.position_z < -0.01) {
          // Support legs...
         RCLCPP_INFO(nh_->get_logger(), "%s","[ForkControlSim]: Forks - activate support legs");
          if (current_report_->status != orunav_msgs::msg::ForkReport::FORK_POSITION_SUPPORT_LEGS)
            {
              reports_mutex_.lock();
              current_report_->status = orunav_msgs::msg::ForkReport::FORK_MOVING_DOWN;
              double current_pos_z = current_report_->state.position_z;
              reports_mutex_.unlock();
              
              send_interpolated_fork_cmds(current_pos_z, msg->state.position_z, 10);
              
              reports_mutex_.lock();
              current_report_->status = orunav_msgs::msg::ForkReport::FORK_POSITION_SUPPORT_LEGS;
              current_report_->state.position_z = msg->state.position_z; // Assume we're down.
              set_operation_status();
              reports_mutex_.unlock();
            }
        }
        else
        {
            // Lower the forks (unless they are already low).
            RCLCPP_INFO(nh_->get_logger(), "%s","[ForkControlSim]: Forks - low");
            if (current_report_->status != orunav_msgs::msg::ForkReport::FORK_POSITION_LOW)
            {
                reports_mutex_.lock();
                current_report_->status = orunav_msgs::msg::ForkReport::FORK_MOVING_DOWN;
                double current_pos_z = current_report_->state.position_z;
                reports_mutex_.unlock();

                send_interpolated_fork_cmds(current_pos_z, msg->state.position_z, 10);

                reports_mutex_.lock();
                current_report_->status = orunav_msgs::msg::ForkReport::FORK_POSITION_LOW;
                current_report_->state.position_z = msg->state.position_z; // Assume we're down.
                set_operation_status();
                reports_mutex_.unlock();
            }
        }

    }



};

#endif
