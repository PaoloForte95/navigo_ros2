#include <algorithm>
#include <assert.h>


//#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include <ecceleron_conversions/conversions.hpp>
#include <gazebo_plugins_oru/gazebo_ros_articulated.h>
namespace gazebo
{


GazeboRosArticulated::GazeboRosArticulated() {}

// Destructor
GazeboRosArticulated::~GazeboRosArticulated() {}

// Load the controller
void GazeboRosArticulated::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "Articulated" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();
    { 
      double length_front_axis, length_rear_axis;
      gazebo_ros_->getParameter<double> ( length_front_axis, "lengthFrontAxis", 1.0 );
      gazebo_ros_->getParameter<double> ( length_rear_axis, "lengthRearAxis", 1.0 );

      wheel_model_.setLengths(length_front_axis, length_rear_axis);
      odometry_model_.setLengths(length_front_axis, length_rear_axis);
    }
    gazebo_ros_->getParameter<double> ( wheel_diameter_front_, "wheelDiameterFront", 0.59 );
    gazebo_ros_->getParameter<double> ( wheel_diameter_rear_, "wheelDiameterRear", 0.59 );

    gazebo_ros_->getParameter<double> ( min_steering_angle_, "minSteeringAngle", -0.7 );
    gazebo_ros_->getParameter<double> ( max_steering_angle_, "maxSteeringAngle", 0.7 );

    gazebo_ros_->getParameter<double> ( steer_torque_, "steerTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( drive_torque_, "driveTorque", 5.0 );
    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_link" );

    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    gazebo_ros_->getParameter<double> ( wheel_acceleration_, "wheelAcceleration", 0 );
    gazebo_ros_->getParameter<double> ( wheel_deceleration_, "wheelDeceleration", wheel_acceleration_ );
    
    gazebo_ros_->getParameter<int> ( nb_axis_rear_, "nbAxisRear", 1 );
    
    { 
      double x, y;
      gazebo_ros_->getParameter<double> ( x, "fl_wheel_pos_x", 0.0 );
      gazebo_ros_->getParameter<double> ( y, "fl_wheel_pos_y", 1.0 );
      wheel_model_.addWheelFrontBody(ecl::mapping::articulated_geometry::DriveWheel(x,y, wheel_diameter_front_/2.));
      gazebo_ros_->getParameter<double> ( x, "fr_wheel_pos_x", 0.0 );
      gazebo_ros_->getParameter<double> ( y, "fr_wheel_pos_y", -1.0 );
      wheel_model_.addWheelFrontBody(ecl::mapping::articulated_geometry::DriveWheel(x,y, wheel_diameter_front_/2.));
      gazebo_ros_->getParameter<double> ( x, "bl_wheel_pos_x", 0.);
      gazebo_ros_->getParameter<double> ( y, "bl_wheel_pos_y", 1.0 );
      wheel_model_.addWheelRearBody(ecl::mapping::articulated_geometry::DriveWheel(x,y, wheel_diameter_rear_/2.));
      gazebo_ros_->getParameter<double> ( x, "br_wheel_pos_x", 0.);
      gazebo_ros_->getParameter<double> ( y, "br_wheel_pos_y", -1.0 );
      wheel_model_.addWheelRearBody(ecl::mapping::articulated_geometry::DriveWheel(x,y, wheel_diameter_rear_/2.));
      //if (nb_axis_rear_ > 1) {
        //gazebo_ros_->getParameter<double> ( x, "bl2_wheel_pos_x", 0.);
        //gazebo_ros_->getParameter<double> ( y, "bl2_wheel_pos_y", 1.0 );
        //wheel_model_.addWheelRearBody(ecl::mapping::articulated_geometry::DriveWheel(x,y, wheel_diameter_rear_/2.));
        //gazebo_ros_->getParameter<double> ( x, "br2_wheel_pos_x", 0.);
        //gazebo_ros_->getParameter<double> ( y, "br2_wheel_pos_y", -1.0 );
        //wheel_model_.addWheelRearBody(ecl::mapping::articulated_geometry::DriveWheel(x,y, wheel_diameter_rear_/2.));
      //}

    }
      
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );

    // Articulated steering joint
    joint_articulated_steer_ = gazebo_ros_->getJoint ( parent, "articulatedSteerJoint", "front_steer_joint" );
    
    // 4WD
    joint_wheel_fl_drive_ = gazebo_ros_->getJoint( parent, "wheelFrontLeftDriveJoint", "front2wheel_fl_joint");
    joint_wheel_fr_drive_ = gazebo_ros_->getJoint( parent, "wheelFrontRightDriveJoint", "front2wheel_fr_joint");
    joint_wheel_bl_drive_ = gazebo_ros_->getJoint( parent, "wheelBackLeftDriveJoint", "rear2wheel_bl_joint");
    joint_wheel_br_drive_ = gazebo_ros_->getJoint( parent, "wheelBackLeftDriveJoint", "rear2wheel_br_joint");
    //if (nb_axis_rear_ > 1) {
      //joint_wheel_bl2_drive_ = gazebo_ros_->getJoint( parent, "wheelBackLeft2DriveJoint", "back_left_wheel2_joint");
      //joint_wheel_br2_drive_ = gazebo_ros_->getJoint( parent, "wheelBackRight2DriveJoint", "back_right_wheel2_joint");
    //}



    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

    joint_articulated_steer_->SetParam ( "fmax", 0, steer_torque_ );
   
    joint_wheel_fl_drive_->SetParam ( "fmax", 0, drive_torque_ );
    joint_wheel_fr_drive_->SetParam ( "fmax", 0, drive_torque_ );
    joint_wheel_bl_drive_->SetParam ( "fmax", 0, drive_torque_ );
    joint_wheel_br_drive_->SetParam ( "fmax", 0, drive_torque_ );
    //if (nb_axis_rear_ > 1) {
      //joint_wheel_bl2_drive_->SetParam ( "fmax", 0, drive_torque_ );
      //joint_wheel_br2_drive_->SetParam ( "fmax", 0, drive_torque_ );
    //}

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_actuator_update_ = parent->GetWorld()->SimTime();
#else
    last_actuator_update_ = parent->GetWorld()->GetSimTime();
#endif

    // Initialize velocity stuff
    alive_ = true;

    if ( this->publishWheelJointState_ ) {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState> ( "joint_states", 1000 );
        ROS_INFO ( "%s: Advertise joint_states!", gazebo_ros_->info() );
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster> ( new tf::TransformBroadcaster() );

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO ( "%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist> ( command_topic_, 1,
                boost::bind ( &GazeboRosArticulated::cmdVelCallback, this, _1 ),
                ros::VoidPtr(), &queue_ );

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe ( so );
    ROS_INFO ( "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str() );

    odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry> ( odometry_topic_, 1 );
    ROS_INFO ( "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str() );

    // start custom queue for diff drive
    this->callback_queue_thread_ = boost::thread ( boost::bind ( &GazeboRosArticulated::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosArticulated::UpdateChild, this ) );

}

void GazeboRosArticulated::publishWheelJointState()
{
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_articulated_steer_ );

    joints.push_back ( joint_wheel_fl_drive_ );
    joints.push_back ( joint_wheel_fr_drive_ );
    joints.push_back ( joint_wheel_bl_drive_ );
    joints.push_back ( joint_wheel_br_drive_ );
    //if (nb_axis_rear_ > 1) {
      //joints.push_back( joint_wheel_bl2_drive_ );
      //joints.push_back( joint_wheel_br2_drive_ );
    //}

    ros::Time current_time = ros::Time::now();
    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints.size() );
    joint_state_.position.resize ( joints.size() );
    joint_state_.velocity.resize ( joints.size() );
    joint_state_.effort.resize ( joints.size() );
    for ( std::size_t i = 0; i < joints.size(); i++ ) {
        joint_state_.name[i] = joints[i]->GetName();
#if GAZEBO_MAJOR_VERSION >= 8
	      joint_state_.position[i] = joints[i]->Position ( 0 );
#else
	      joint_state_.position[i] = joints[i]->GetAngle ( 0 ).Radian();
#endif
        joint_state_.velocity[i] = joints[i]->GetVelocity ( 0 );
        joint_state_.effort[i] = joints[i]->GetForce ( 0 );
    }
    joint_state_publisher_.publish ( joint_state_ );
}

void GazeboRosArticulated::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    std::vector<physics::JointPtr> joints;
    joints.push_back ( joint_articulated_steer_ );

    joints.push_back ( joint_wheel_fl_drive_ );
    joints.push_back ( joint_wheel_fr_drive_ );
    joints.push_back ( joint_wheel_bl_drive_ );
    joints.push_back ( joint_wheel_br_drive_ );
    //if (nb_axis_rear_ > 1) {
      //joints.push_back( joint_wheel_bl2_drive_ );
      //joints.push_back( joint_wheel_br2_drive_ );
    //}

    for ( std::size_t i = 0; i < joints.size(); i++ ) {
      std::string frame = gazebo_ros_->resolveTF ( joints[i]->GetName() );
      std::string parent_frame = gazebo_ros_->resolveTF ( joints[i]->GetParent()->GetName() );

#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = joints[i]->GetChild()->RelativePose();
#else
        ignition::math::Pose3d pose = joints[i]->GetChild()->GetRelativePose().Ign();
#endif
   
	      tf::Quaternion qt ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
	      tf::Vector3 vt ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        tf::Transform transform ( qt, vt );
        transform_broadcaster_->sendTransform ( tf::StampedTransform ( transform, current_time, parent_frame, frame ) );
    }

}
// Update the controller
void GazeboRosArticulated::UpdateChild()
{
    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_actuator_update_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        double target_wheel_speed = cmd_.speed;// / ( wheel_diameter_ / 2.0 );
        double target_steering_angle_speed = cmd_.angle;

        motorController ( target_wheel_speed, target_steering_angle_speed, seconds_since_last_update );

        //        ROS_INFO_STREAM("v = " << target_wheel_speed << " w = " << target_steering_angle_speed);

        last_actuator_update_ += common::Time ( update_period_ );
    }
}

inline void jointPositionPDControl(physics::JointPtr joint,
                                   double target, double kp, double kd) {
#if GAZEBO_MAJOR_VERSION >= 8
    double current = joint->Position(0);
#else
    double current = joint->GetAngle(0).Radian();
#endif
  double v = kp * (target - current) - kd * joint->GetVelocity(0);
  // ROS_INFO_STREAM("current : " << current << " target : " << target << " output v : " << v);
  joint->SetParam ( "vel", 0, v);
}

void GazeboRosArticulated::motorController ( double target_speed, double target_steering_speed, double dt )
{
  // Each joint is controlled independently (instead of having a set of parallel links). 

#if GAZEBO_MAJOR_VERSION >= 8
  double steering_angle = joint_articulated_steer_->Position(0);
#else
  double steering_angle = joint_articulated_steer_->GetAngle(0).Radian();
#endif

  double steering_angle_velocity = joint_articulated_steer_->GetVelocity(0);

  // Check that the joints limits are meet, this are not handled well by the simulator
  if (steering_angle > max_steering_angle_) {
    if (target_steering_speed > 0) {
      target_steering_speed = 0;
    }
  }
  if (steering_angle < min_steering_angle_) {
    if (target_steering_speed < 0) {
      target_steering_speed = 0;
    }
  }

  wheel_model_.updateWheels(steering_angle);

  
  // Run a simple positioning control on the steering angle.
  jointPositionPDControl(joint_articulated_steer_, steering_angle, 10, 1);

  joint_articulated_steer_->SetParam( "vel", 0, target_steering_speed );
  
  double target_wheel_speed = cmd_.speed; // Note this is the linear speed in m/s, we need to update this to rotation speed considering the wheel radius

  // Front
  joint_wheel_fl_drive_->SetParam( "vel", 0, wheel_model_.front_wheels[0].convertLinearSpeedToRotationalJointSpeed() * target_speed);
  joint_wheel_fr_drive_->SetParam( "vel", 0, wheel_model_.front_wheels[1].convertLinearSpeedToRotationalJointSpeed() * target_speed);

  // Rear
  joint_wheel_bl_drive_->SetParam( "vel", 0, wheel_model_.rear_wheels[0].convertLinearSpeedToRotationalJointSpeed() * target_speed);
  joint_wheel_br_drive_->SetParam( "vel", 0, wheel_model_.rear_wheels[1].convertLinearSpeedToRotationalJointSpeed() * target_speed);

  //if (nb_axis_rear_ > 1) {
    //joint_wheel_bl2_drive_->SetParam( "vel", 0, wheel_model_.rear_wheels[2].convertLinearSpeedToRotationalJointSpeed() * target_speed);
    //joint_wheel_br2_drive_->SetParam( "vel", 0, wheel_model_.rear_wheels[3].convertLinearSpeedToRotationalJointSpeed() * target_speed);
  //}
}

// Finalize the controller
void GazeboRosArticulated::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosArticulated::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    cmd_.speed = cmd_msg->linear.x;
    cmd_.angle = cmd_msg->angular.z;
}

void GazeboRosArticulated::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosArticulated::UpdateOdometryEncoder()
{
  // This is currently based on old code with a combined steer and drive wheel (e.g. snowwhite / cititruck)
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double step_time = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;
#if GAZEBO_MAJOR_VERSION >= 8
    double steering_angle = joint_articulated_steer_->Position(0);
#else
    double steering_angle = joint_articulated_steer_->GetAngle(0).Radian();
#endif
    double steering_angle_velocity = joint_articulated_steer_->GetVelocity(0);
    double drive_vel = 0.5*(joint_wheel_fl_drive_->GetVelocity(0) + 
       joint_wheel_fr_drive_->GetVelocity(0));


    // Distance travelled (this is utilizing only the front wheels...)
    double drive_dist = drive_vel * step_time * wheel_diameter_front_ / 2.;

    ecl::mapping::PoseVector2d velocities = odometry_model_.getVelocityPoseFront(steering_angle, drive_vel, steering_angle_velocity);
    odometry_model_.integrateCommand(steering_angle, drive_vel, steering_angle_velocity, step_time);

    ecl::mapping::PoseVector2d odom_pose = odometry_model_.getFrontAxisPoseVector2d();
    
    odom_.pose.pose = ecl::mapping::conversions::toPoseMsg(odometry_model_.getFrontAxisPoseVector2d());

    odom_.twist.twist.linear.x = velocities[0];
    odom_.twist.twist.linear.y = velocities[1];
    odom_.twist.twist.angular.z = velocities[2];
}

void GazeboRosArticulated::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );
    }
    if ( odom_source_ == WORLD ) {
        // getting data form gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent->WorldPose();
#else
        ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
      	qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
	      vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
	      ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
	      linear = parent->WorldLinearVel();
	      odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
#else
	      linear = parent->GetWorldLinearVel().Ign();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
#endif

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
	      odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
	      odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    tf::Transform base_footprint_to_odom ( qt, vt );
    transform_broadcaster_->sendTransform (
        tf::StampedTransform ( base_footprint_to_odom, current_time,
                               odom_frame, base_footprint_frame ) );


    // set covariance - TODO, fix this(!)
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosArticulated )
}

