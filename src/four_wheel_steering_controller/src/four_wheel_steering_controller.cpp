#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "controller_interface/controller_interface_base.hpp"
#include "/home/charith-2204/ranger_mini_v2/src/four_wheel_steering_controller/include/four_wheel_steering_controller/four_wheel_steering_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/chrono/process_cpu_clocks.hpp>
#include <boost/chrono/system_clocks.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <boost/chrono/thread_clock.hpp>
#include <rclcpp/time.hpp> 


// namespace

// {  

// void reset_controller_reference_msg(
//   const geometry_msgs::msg::Twist& command,
//   const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
// {
//   command_struct_twist_.stamp = node->get_clock()->now();
//   command_struct_twist_.x = std::numeric_limits<double>::quiet_NaN();
//   command_struct_twist_.linear.y = std::numeric_limits<double>::quiet_NaN();
//   command_struct_twist_.z = std::numeric_limits<double>::quiet_NaN();
//   command_struct_twist_.angular.x = std::numeric_limits<double>::quiet_NaN();
//   command_struct_twist_.y = std::numeric_limits<double>::quiet_NaN();
//   command_struct_twist_.z = std::numeric_limits<double>::quiet_NaN();
// }

// } 

namespace four_wheel_steering_controller
{
FourWheelSteeringController::FourWheelSteeringController()
: controller_interface::ChainableControllerInterface(),

  publish_period_(std::chrono::duration<double>(0.02))
  
{

}
  
controller_interface::CallbackReturn FourWheelSteeringController::on_init()
{
  // try
  // {
  //   //swerve_param_listener =std::make_shared<FourWheelSteeringController::ParamListener>(get_node());

  // }
  // catch (const std::exception & e)
  // {
  //   fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering on_init function successful");

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn FourWheelSteeringController::set_interface_numbers(
  size_t nr_state_itfs = 8, size_t nr_cmd_itfs = 8, size_t nr_ref_itfs = 2)
{
  nr_state_itfs_ = nr_state_itfs;
  nr_cmd_itfs_ = nr_cmd_itfs;
  nr_ref_itfs_ = nr_ref_itfs;

  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering set_interface_numbers(inside configure_odometry ---> on_configure) function successful");

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn FourWheelSteeringController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
  {
  // swerve_params_ = swerve_param_listener_->get_params();
  // odometry.set_velocity_rolling_window_size(swerve_params_.velocity_rolling_window_size);
  
  front_wheels_state_names_ = {"front_right_wheel_joint", "front_left_wheel_joint"};
  rear_wheels_state_names_ = {"rear_right_wheel_joint", "rear_left_wheel_joint"};
  front_steering_state_names_ = {"front_right_wheel_steering_joint", "front_left_wheel_steering_joint"};
  rear_steering_state_names_ = {"rear_right_wheel_steering_joint", "rear_left_wheel_steering_joint"};

  const double wheel_base_ =0.494; //0.492; //  // needs to figure out
  const double track_ = 0.364; //0.39; //.364;                     // needs to figure out
  const double wheel_radius_ = 0.085; 
  const double wheel_steering_y_offset_ = -0.0702;

  pose_covariance_diagonal = {0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0};
  twist_covariance_diagonal = {0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0};

  if (odometry.setWheelParams(track_,wheel_steering_y_offset_,wheel_radius_,wheel_base_))
   {
     RCLCPP_INFO(get_node()->get_logger(), "four wheel steering odom configure successful");
   }
  
  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  //Odometry related:
  
  publish_rate_= 50.0;
  RCLCPP_INFO(get_node()->get_logger(), "Controller state will be published at %f Hz",publish_rate_);
  //publish_period = 1.0/publish_rate_;
  odometry.setVelocityRollingWindowSize(velocity_rolling_window_size);
  
  //Twist command related:
  reference_timeout=1.0;
  cmd_vel_timeout_=1.0; //seconds
  base_frame_id_ = "base_link";
  odom_frame_id_ = "odom";
  isRunning = true;
  
  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Subscribers
  ref_timeout_ = rclcpp::Duration::from_seconds(reference_timeout);

  vel_subscriber_twist_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", subscribers_qos,
    std::bind(&FourWheelSteeringController::cmdVelCallback, this, std::placeholders::_1));

  // steering_subscriber_ = get_node()->create_subscription<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>(
  //   "~/cmd_four_wheel_steering", subscribers_qos,
  //   std::bind(
  //     &FourWheelSteeringController::cmdFourWheelSteeringCallback, this, std::placeholders::_1));

   try
  {
    // Odom state publisher
    odom_s_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "/odometry", rclcpp::SystemDefaultsQoS());
    odom_pub_ = std::make_unique<ControllerStatePublisherOdom>(odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  odom_pub_->lock();
  odom_pub_->msg_.header.stamp = get_node()->now();
  odom_pub_->msg_.header.frame_id = odom_frame_id_;
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;

  auto & covariance = odom_pub_->msg_.twist.covariance;
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    covariance[diagonal_index] = pose_covariance_diagonal[index];
    covariance[diagonal_index] = twist_covariance_diagonal[index];
  }
  odom_pub_->unlock();

  try
  {
    // Tf State publisher
    tf_odom_s_publisher_ = get_node()->create_publisher<ControllerStateMsgTf>(
      "/tf_odometry", rclcpp::SystemDefaultsQoS());
    tf_odom_pub_ =
      std::make_unique<ControllerStatePublisherTf>(tf_odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  tf_odom_pub_->lock();
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].header.stamp = get_node()->now();
  tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->unlock();


  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering on_configure function successful");

           
  return controller_interface::CallbackReturn::SUCCESS;

}


bool FourWheelSteeringController::update_odometry(const rclcpp::Duration & /*period*/)
{
  
  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering update_odometry function successful");
  return true;
}


controller_interface::InterfaceConfiguration FourWheelSteeringController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(nr_cmd_itfs_);

  for (size_t i = 0; i < front_wheels_state_names_.size(); i++)
  {
    command_interfaces_config.names.push_back(
      front_wheels_state_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
  }
    for (size_t i = 0; i < rear_wheels_state_names_.size(); i++)
  {
    command_interfaces_config.names.push_back(
      rear_wheels_state_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (size_t i = 0; i < front_steering_state_names_.size(); i++)
  {
    command_interfaces_config.names.push_back(
      front_steering_state_names_[i] + "/" + hardware_interface::HW_IF_POSITION); 
  }
  for (size_t i = 0; i < rear_steering_state_names_.size(); i++)
  {
    command_interfaces_config.names.push_back(
      rear_steering_state_names_[i] + "/" + hardware_interface::HW_IF_POSITION);  
  }
  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering command_interface_configuration function successful");

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
FourWheelSteeringController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(nr_state_itfs_);

    for (size_t i = 0; i < front_wheels_state_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
      front_wheels_state_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    for (size_t i = 0; i < rear_wheels_state_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
      rear_wheels_state_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    for (size_t i = 0; i < front_steering_state_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
      front_steering_state_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
    }
    for (size_t i = 0; i < rear_steering_state_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
      rear_steering_state_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
    }

  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering state_interface_configuration function successful");

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
FourWheelSteeringController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(2, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(2);

  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("linear/") + hardware_interface::HW_IF_VELOCITY,
    &reference_interfaces_[0]));

  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("angular/") + hardware_interface::HW_IF_POSITION,
    &reference_interfaces_[1]));
  
  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering on_export_reference_interfaces function successful");

  return reference_interfaces;
}


controller_interface::CallbackReturn FourWheelSteeringController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());
  isRunning = true;
  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering on_activate function successful");

  return controller_interface::CallbackReturn::SUCCESS;
} 


controller_interface::CallbackReturn FourWheelSteeringController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // for (size_t i = 0; i < nr_cmd_itfs_; ++i)
  // {
  //   command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  // }
  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering on_deactivate function successful");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FourWheelSteeringController::update_reference_from_subscribers()
{
  // Move functionality to the `update_and_write_commands` because of the missing arguments in
  // humble - otherwise issues with multiple time-sources might happen when working with simulators
  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering update_reference_from_subscribers function successful");

  return controller_interface::return_type::OK;
}

controller_interface::return_type FourWheelSteeringController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  updateOdometry(time);
  updateCommand(time, period);

  RCLCPP_INFO(get_node()->get_logger(), "-------four wheel steering update_and_write_commands function successful------");

  return controller_interface::return_type::OK;
}



void FourWheelSteeringController::updateOdometry(const rclcpp::Time& time)
{ 
  double fl_steering=0.0;
  double rl_steering=0.0;
  double fr_steering=0.0;
  double rr_steering=0.0;

  double fl_speed=0.0;
  double rl_speed=0.0;
  double fr_speed=0.0;
  double rr_speed=0.0;

  CommandTwist curr_cmd_twist = *(command_twist_.readFromRT());

 if (fabs(curr_cmd_twist.lin_y) > 0.001 && fabs(curr_cmd_twist.lin_x) < 0.001 )
 {
  double sign =copysign(1.0,curr_cmd_twist.lin_y);
  fl_speed = sign*reverse_fr*state_interfaces_[STATE_TRACTION_FRONT_RIGHT_WHEEL].get_value();
  rl_speed = sign*reverse_fl*state_interfaces_[STATE_TRACTION_FRONT_LEFT_WHEEL].get_value();
  fr_speed = sign*reverse_rr*state_interfaces_[STATE_TRACTION_REAR_RIGHT_WHEEL].get_value();
  rr_speed = sign*reverse_rl*state_interfaces_[STATE_TRACTION_REAR_LEFT_WHEEL].get_value();

  fl_steering = state_interfaces_[STATE_STEER_FRONT_RIGHT_WHEEL].get_value();
  rl_steering = state_interfaces_[STATE_STEER_FRONT_LEFT_WHEEL].get_value();
  fr_steering = state_interfaces_[STATE_STEER_REAR_RIGHT_WHEEL].get_value();
  rr_steering = state_interfaces_[STATE_STEER_REAR_LEFT_WHEEL].get_value();

 }
 else
 {
  fr_speed = reverse_fr*state_interfaces_[STATE_TRACTION_FRONT_RIGHT_WHEEL].get_value();
  fl_speed = reverse_fl*state_interfaces_[STATE_TRACTION_FRONT_LEFT_WHEEL].get_value();
  rr_speed = reverse_rr*state_interfaces_[STATE_TRACTION_REAR_RIGHT_WHEEL].get_value();
  rl_speed = reverse_rl*state_interfaces_[STATE_TRACTION_REAR_LEFT_WHEEL].get_value();

  fr_steering = state_interfaces_[STATE_STEER_FRONT_RIGHT_WHEEL].get_value();
  fl_steering = state_interfaces_[STATE_STEER_FRONT_LEFT_WHEEL].get_value();
  rr_steering = state_interfaces_[STATE_STEER_REAR_RIGHT_WHEEL].get_value();
  rl_steering = state_interfaces_[STATE_STEER_REAR_LEFT_WHEEL].get_value(); 
 }





  // // COMPUTE AND PUBLISH ODOMETRY
  // const double fr_speed = reverse_fr*state_interfaces_[STATE_TRACTION_FRONT_RIGHT_WHEEL].get_value();
  // const double fl_speed = reverse_fl*state_interfaces_[STATE_TRACTION_FRONT_LEFT_WHEEL].get_value();
  // const double rr_speed = reverse_rr*state_interfaces_[STATE_TRACTION_REAR_RIGHT_WHEEL].get_value();
  // const double rl_speed = reverse_rl*state_interfaces_[STATE_TRACTION_REAR_LEFT_WHEEL].get_value();

  if (std::isnan(fl_speed) || std::isnan(fr_speed)
      || std::isnan(rl_speed) || std::isnan(rr_speed))
    
    return;
    
  // const double fr_steering = state_interfaces_[STATE_STEER_FRONT_RIGHT_WHEEL].get_value();
  // const double fl_steering = state_interfaces_[STATE_STEER_FRONT_LEFT_WHEEL].get_value();
  // const double rr_steering = state_interfaces_[STATE_STEER_REAR_RIGHT_WHEEL].get_value();
  // const double rl_steering = state_interfaces_[STATE_STEER_REAR_LEFT_WHEEL].get_value();

  // const double fl_steering = state_interfaces_[STATE_STEER_FRONT_RIGHT_WHEEL].get_value();
  // const double rl_steering = state_interfaces_[STATE_STEER_FRONT_LEFT_WHEEL].get_value();
  // const double fr_steering = state_interfaces_[STATE_STEER_REAR_RIGHT_WHEEL].get_value();
  // const double rr_steering = state_interfaces_[STATE_STEER_REAR_LEFT_WHEEL].get_value();

 

  RCLCPP_INFO(get_node()->get_logger(),
                             " writing values in state_interfaces_. \n"
                             " vel_right_front: %f \n"
                             " vel_left_front: %f \n"
                             " vel_right_rear: %f \n"
                             " vel_left_rear: %f \n"
                             " front_right_steering: %f \n"
                             " front_left_steering: %f \n"
                             " rear_right_steering: %f \n"
                             " rear_left_steering: %f \n",
                             fl_speed,fr_speed,rr_speed,rl_speed,
                             fr_steering,fl_steering,rr_steering,rl_steering);

  if (std::isnan(fl_steering) || std::isnan(fr_steering)
      || std::isnan(rl_steering) || std::isnan(rr_steering))
    return;
  double front_steering_pos = 0.0;
  if(fabs(fl_steering) > 0.001 || fabs(fr_steering) > 0.001)
  {
    front_steering_pos = atan(2*tan(fl_steering)*tan(fr_steering)/
                                    (tan(fl_steering) + tan(fr_steering)));
  }
  double rear_steering_pos = 0.0;
  if(fabs(rl_steering) > 0.001 || fabs(rr_steering) > 0.001)
  {
    rear_steering_pos = atan(2*tan(rl_steering)*tan(rr_steering)/
                                    (tan(rl_steering) + tan(rr_steering)));
  }

  //RCLCPP_INFO(get_node()->get_logger(),"rl_steering %f rr_steering %f rear_steering_pos %f",rl_steering,rr_steering,rear_steering_pos);
  // Estimate linear and angular velocity using joint information

 if (fabs(curr_cmd_twist.lin_y) > 0.001 && fabs(curr_cmd_twist.lin_x) < 0.001 )
 {
    odometry.update(fl_speed, fr_speed, rl_speed, rr_speed,front_steering_pos, rear_steering_pos, time,true);
 }
 else
 {
    odometry.update(fl_speed, fr_speed, rl_speed, rr_speed,front_steering_pos, rear_steering_pos, time);
 }

 rclcpp::Clock clock;
  // Publish odometry message
  if (last_state_publish_time_ + publish_period_ < clock.now())
  { 
   
  //  RCLCPP_INFO(get_node()->get_logger(),
  //           "Added values to command."
  //           " Last state publish time: %.6f s\n"
  //                       " Clock now: %.6f s\n"
  //                       "Time : %.6f s\n",
  //           last_state_publish_time_.seconds(),
  //           clock.now().seconds(),
  //           time.seconds());

    last_state_publish_time_ += publish_period_;
    // Compute and store orientation info


    geometry_msgs::msg::Quaternion orientation;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odometry.getHeading());
    orientation = tf2::toMsg(q);
    

    // Populate odom message and publish
    if (odom_pub_->trylock())
    {
      odom_pub_->msg_.header.stamp = time;
      odom_pub_->msg_.pose.pose.position.x = odometry.getX();
      odom_pub_->msg_.pose.pose.position.y = odometry.getY();
      odom_pub_->msg_.pose.pose.orientation = orientation;
      odom_pub_->msg_.twist.twist.linear.x  = odometry.getLinearX();
      odom_pub_->msg_.twist.twist.linear.y  = odometry.getLinearY();
      odom_pub_->msg_.twist.twist.angular.z = odometry.getAngular();
      odom_pub_->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (enable_odom_tf_ && tf_odom_pub_->trylock())
    {
      geometry_msgs::msg::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
      odom_frame.header.stamp = time;
      odom_frame.transform.translation.x = odometry.getX();
      odom_frame.transform.translation.y = odometry.getY();
      odom_frame.transform.rotation = orientation;
      tf_odom_pub_->unlockAndPublish();
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering updateodometry function successful");
}


void FourWheelSteeringController::updateCommand(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  Command* cmd;
  CommandTwist curr_cmd_twist = *(command_twist_.readFromRT());

  cmd = &curr_cmd_twist;
  enable_twist_cmd_ = true;


  const double dt = (time.nanoseconds() - cmd->stamp.nanoseconds())/ 1e9 ;

  // Brake if cmd_vel has timeout://
  if (dt > cmd_vel_timeout_)
  {
    curr_cmd_twist.lin_x = 0.0000;
    curr_cmd_twist.lin_y = 0.0000;
    curr_cmd_twist.ang = 0.0000;
  }

  const double cmd_dt(period.seconds());
  const double angular_speed = odometry.getAngular();
  const double steering_track = track_-2*wheel_steering_y_offset_;

  RCLCPP_INFO(get_node()->get_logger(),"angular_speed %f wheel_radius_ %f",angular_speed,wheel_radius_);

  
  double vel_left_front = 0.000, vel_right_front = 0.000;
  double vel_left_rear = 0.000, vel_right_rear = 0.000;
  double front_left_steering = 0.000, front_right_steering = 0.000;
  double rear_left_steering = 0.000, rear_right_steering = 0.000;

 
 
    // Limit velocities and accelerations:
    limiter_lin_.limit_lin(curr_cmd_twist.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
    limiter_ang_.limit_ang(curr_cmd_twist.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);
    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd_twist;

    
    
    // Compute wheels velocities:

    if( (fabs(curr_cmd_twist.ang) < 0.001) ||                                              // No rotation
        (fabs(curr_cmd_twist.lin_x) >= 0.001 && fabs(curr_cmd_twist.lin_y) >= 0.001) ) //parallel steering
    {
      const double sign = copysign(1.0, curr_cmd_twist.lin_x);
      reverse_fl=1;
      reverse_fr=1;
      reverse_rl=-1;
      reverse_rr=-1;
      vel_left_front  =reverse_fl*sign * std::hypot((curr_cmd_twist.lin_x),(curr_cmd_twist.lin_y)) / wheel_radius_;
      vel_right_front =reverse_fr*sign * std::hypot((curr_cmd_twist.lin_x),(curr_cmd_twist.lin_y)) / wheel_radius_ ;
      vel_left_rear   =reverse_rl*sign * std::hypot((curr_cmd_twist.lin_x),(curr_cmd_twist.lin_y)) / wheel_radius_;
      vel_right_rear  =reverse_rr*sign * std::hypot((curr_cmd_twist.lin_x),(curr_cmd_twist.lin_y)) / wheel_radius_;
    }
    else if ( (fabs(curr_cmd_twist.lin_x) < 0.001 && fabs(curr_cmd_twist.lin_y) < 0.001) )  // spining
    {
      const double sign = copysign(1.0, curr_cmd_twist.ang);
      reverse_fl=-1;
      reverse_fr=1;
      reverse_rl=1;
      reverse_rr=-1;
      
      vel_left_front  = reverse_fl*sign * std::hypot((curr_cmd_twist.ang*steering_track/2),(wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_;                   
      vel_right_front =reverse_fr*sign * std::hypot((curr_cmd_twist.ang*steering_track/2),(wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_;               
      vel_left_rear = reverse_rl*sign * std::hypot((curr_cmd_twist.ang*steering_track/2),(wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_;               
      vel_right_rear =reverse_rr*sign * std::hypot((curr_cmd_twist.ang*steering_track/2),(wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_;
    }    
    else  //Dual Ackerman
    {
       double sign=1.0;
       if (fabs(curr_cmd_twist.lin_y) < 0.001)
       {
        sign = copysign(sign, curr_cmd_twist.lin_x);
        reverse_fl=1;
        reverse_fr=1;
        reverse_rl=-1;
        reverse_rr=-1;
       }
       else
       {
        sign = copysign(sign, curr_cmd_twist.ang);
        reverse_fl=-1;
        reverse_fr=1;
        reverse_rl=1;
        reverse_rr=-1;
       }

        vel_left_front  =reverse_fl*sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),(curr_cmd_twist.lin_y + wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_;
        vel_right_front =reverse_fr*sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),(curr_cmd_twist.lin_y + wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_;
        vel_left_rear =reverse_rl*sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),(curr_cmd_twist.lin_y - wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_;
        vel_right_rear =reverse_rr*sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),(curr_cmd_twist.lin_y - wheel_base_*curr_cmd_twist.ang/2.0)) / wheel_radius_;
    }

    if( command_interfaces_.size()== 8) {
      command_interfaces_[CMD_TRACTION_FRONT_RIGHT_WHEEL].set_value(vel_right_front);
      command_interfaces_[CMD_TRACTION_FRONT_LEFT_WHEEL].set_value(vel_left_front);
      command_interfaces_[CMD_TRACTION_REAR_RIGHT_WHEEL].set_value(vel_right_rear);
      command_interfaces_[CMD_TRACTION_REAR_LEFT_WHEEL].set_value(vel_left_rear);
    }

    if ((fabs(curr_cmd_twist.lin_x) < 0.001) &&
        (fabs(curr_cmd_twist.lin_y) < 0.001) &&
        (fabs(curr_cmd_twist.ang) < 0.001) )
    {
      return;
    }

   /*--------- Compute steering angles [Edit : Achala] --------------*/
    // Parallel Steering
    if( (fabs(curr_cmd_twist.ang) < 0.001) ||                                              // No rotation
        (fabs(curr_cmd_twist.lin_x) >= 0.001 && fabs(curr_cmd_twist.lin_y) >= 0.001) )     // x,y velocities both present
    { 
      
      double steering_angle = atan(curr_cmd_twist.lin_y / curr_cmd_twist.lin_x);
      front_left_steering = steering_angle;
      front_right_steering = steering_angle;
      rear_left_steering = steering_angle; 
      rear_right_steering = steering_angle;  
        
    }
    // Spinning
    else if ( (fabs(curr_cmd_twist.lin_x) < 0.001 && fabs(curr_cmd_twist.lin_y) < 0.001) )     // No linear motion
    {
      double spining_steer_angle = atan(wheel_base_/steering_track);
      
      front_left_steering = -spining_steer_angle;
      front_right_steering = spining_steer_angle;
      rear_left_steering = spining_steer_angle;
      rear_right_steering = -spining_steer_angle;
    }
    // Dual ackerman
    else
    {
       front_left_steering = atan((curr_cmd_twist.lin_y + curr_cmd_twist.ang*wheel_base_/2) / (curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2));
       front_right_steering = atan((curr_cmd_twist.lin_y + curr_cmd_twist.ang*wheel_base_/2)/ (curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2));
       rear_left_steering = atan((curr_cmd_twist.lin_y - curr_cmd_twist.ang*wheel_base_/2) / (curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2));
       rear_right_steering = atan((curr_cmd_twist.lin_y - curr_cmd_twist.ang*wheel_base_/2) / (curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2));  
    }

    if( command_interfaces_.size()== 8) {
      command_interfaces_[CMD_STEER_FRONT_RIGHT_WHEEL].set_value(front_right_steering);
      command_interfaces_[CMD_STEER_FRONT_LEFT_WHEEL].set_value(front_left_steering);
      command_interfaces_[CMD_STEER_REAR_RIGHT_WHEEL].set_value(rear_right_steering);
      command_interfaces_[CMD_STEER_REAR_LEFT_WHEEL].set_value(rear_left_steering);
    }
    
  // RCLCPP_INFO(get_node()->get_logger(),
  //                            " writing values to command_interfaces_. \n"
  //                            " vel_right_front: %f \n"
  //                            " vel_left_front: %f \n"
  //                            " vel_right_rear: %f \n"
  //                            " vel_left_rear: %f \n"
  //                            " front_right_steering: %f \n"
  //                            " front_left_steering: %f \n"
  //                            " rear_right_steering: %f \n"
  //                            " rear_left_steering: %f \n",
  //                            vel_right_front,vel_left_front,vel_right_rear,vel_left_rear,
  //                            front_right_steering,front_left_steering,rear_right_steering,rear_left_steering);

  RCLCPP_INFO(get_node()->get_logger(), "four wheel steering updatecommand function successful");

}

void FourWheelSteeringController::cmdVelCallback(const geometry_msgs::msg::Twist& command)
{
  if (isRunning)
  {
    // Check for NaN values
    if (std::isnan(command.angular.z) || std::isnan(command.linear.x))
    {
      RCLCPP_INFO(get_node()->get_logger(), "Received NaN in geometry_msgs::Twist. Ignoring command.");
      return;
    }

    // Update command structure
    command_struct_twist_.ang   = command.angular.z;
    command_struct_twist_.lin_x   = command.linear.x;
    command_struct_twist_.lin_y   = command.linear.y;
    command_struct_twist_.stamp = get_node()->now();

    // Write to non-real-time buffer
    command_twist_.writeFromNonRT(command_struct_twist_);
    RCLCPP_INFO(get_node()->get_logger(),
                             " Added values to command. \n"
                             " Ang: %f \n"
                             " Lin x: %f \n"
                             " Lin y: %f \n"
                             " Stamp: %f ",
                             command_struct_twist_.ang, command_struct_twist_.lin_x,
                             command_struct_twist_.lin_y, command_struct_twist_.stamp.seconds());
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(),"Can't accept new commands. Controller is not running.");
  }
}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  four_wheel_steering_controller::FourWheelSteeringController,
  controller_interface::ChainableControllerInterface)
