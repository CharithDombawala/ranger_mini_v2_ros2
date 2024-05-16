#ifndef FOUR_WHEEL_STEERING_CONTROLLER__FOUR_WHEEL_STEERING_CONTROLLER_HPP_
#define FOUR_WHEEL_STEERING_CONTROLLER__FOUR_WHEEL_STEERING_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

#include "four_wheel_steering_controller/speed_limiter.hpp"
#include "four_wheel_steering_controller/odometry.h"
#include "four_wheel_steering_controller/visibility_control.hpp"

#include "control_msgs/msg/steering_controller_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <gazebo_msgs/msg/link_states.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace four_wheel_steering_controller 
{

// name constants for state interfacesc
static constexpr size_t STATE_TRACTION_FRONT_RIGHT_WHEEL = 0;
static constexpr size_t STATE_TRACTION_FRONT_LEFT_WHEEL = 1;
static constexpr size_t STATE_TRACTION_REAR_RIGHT_WHEEL = 2;
static constexpr size_t STATE_TRACTION_REAR_LEFT_WHEEL = 3;
static constexpr size_t STATE_STEER_FRONT_RIGHT_WHEEL = 4;
static constexpr size_t STATE_STEER_FRONT_LEFT_WHEEL = 5;
static constexpr size_t STATE_STEER_REAR_RIGHT_WHEEL = 6;
static constexpr size_t STATE_STEER_REAR_LEFT_WHEEL = 7;

// name constants for command interfaces
static constexpr size_t CMD_TRACTION_FRONT_RIGHT_WHEEL = 0;
static constexpr size_t CMD_TRACTION_FRONT_LEFT_WHEEL = 1;
static constexpr size_t CMD_TRACTION_REAR_RIGHT_WHEEL = 2;
static constexpr size_t CMD_TRACTION_REAR_LEFT_WHEEL = 3;
static constexpr size_t CMD_STEER_FRONT_RIGHT_WHEEL = 4;
static constexpr size_t CMD_STEER_FRONT_LEFT_WHEEL = 5;
static constexpr size_t CMD_STEER_REAR_RIGHT_WHEEL = 6;
static constexpr size_t CMD_STEER_REAR_LEFT_WHEEL = 7;

static constexpr size_t NR_STATE_ITFS = 8;
static constexpr size_t NR_CMD_ITFS = 8;
static constexpr size_t NR_REF_ITFS = 2;


class FourWheelSteeringController : public controller_interface::ChainableControllerInterface
                                   
{
public:
  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC FourWheelSteeringController();

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_init() override;

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
 
  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::return_type
  update_reference_from_subscribers() override;

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::return_type
  update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
  using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;

  FOUR_WHEEL_STEERING_CONTROLLER__VISIBILITY_LOCAL void cmdVelCallback(const geometry_msgs::msg::Twist& command);
  void updateOdometry(const rclcpp::Time& time);
  void updateCommand(const rclcpp::Time& time, const rclcpp::Duration& period);
  void callback(const gazebo_msgs::msg::LinkStates &msg);

  
protected:

  controller_interface::CallbackReturn set_interface_numbers(size_t nr_state_itfs, size_t nr_cmd_itfs, size_t nr_ref_itfs);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_twist_ = nullptr;
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr gazebo_pose_subscription_ = nullptr;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);// 0ms
  
  using ControllerStatePublisherOdom = realtime_tools::RealtimePublisher<ControllerStateMsgOdom>;
  using ControllerStatePublisherTf = realtime_tools::RealtimePublisher<ControllerStateMsgTf>;

  rclcpp::Publisher<ControllerStateMsgOdom>::SharedPtr odom_s_publisher_;
  rclcpp::Publisher<ControllerStateMsgTf>::SharedPtr tf_odom_s_publisher_;
  rclcpp::Publisher<ControllerStateMsgOdom>::SharedPtr odom2_s_publisher_;

  size_t nr_state_itfs_;
  size_t nr_cmd_itfs_;
  size_t nr_ref_itfs_;

  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  
  SpeedLimiter limiter_lin_;
  SpeedLimiter limiter_ang_;

  bool isRunning = false;

  double publish_rate_ = 50.0;
  bool use_stamped_vel_ = true;

  const double wheel_base_ = 0.46; //0.494;  // needs to figure out
  const double track_ = 0.412; //0.364;                     // needs to figure out
  const double wheel_radius_ = 0.09; //0.085; 
  const double wheel_steering_y_offset_ = 0.0;

  std::vector<std::string> rear_wheels_state_names_ = {"front_right_wheel_joint", "front_left_wheel_joint"};
  std::vector<std::string> front_wheels_state_names_ = {"rear_right_wheel_joint", "rear_left_wheel_joint"};
  std::vector<std::string> rear_steering_state_names_ = {"front_right_wheel_steering_joint", "front_left_wheel_steering_joint"};
  std::vector<std::string> front_steering_state_names_ = {"rear_right_wheel_steering_joint", "rear_left_wheel_steering_joint"};

  std::vector<double> pose_covariance_diagonal ={0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0};
  std::vector<double> twist_covariance_diagonal = {0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0};
  
private:
  
  rclcpp::Duration publish_period_;
  rclcpp::Time last_time_;
  rclcpp::Time current_time_;
  rclcpp::Time last_state_publish_time_;

  /// Velocity command related:

  struct Command
  {
    rclcpp::Time stamp;

    Command() : stamp(0.0) {}
  };
  struct CommandTwist : Command
  {
    double lin_x;
    double lin_y;
    double ang;

    CommandTwist() : lin_x(0.0), lin_y(0.0), ang(0.0) {}
  };

  realtime_tools::RealtimeBuffer<CommandTwist> command_twist_;
  CommandTwist command_struct_twist_;

  double position_x;
  double position_y;
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry> > odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage> > tf_odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry> > odom2_pub_;
  Odometry odometry;


  CommandTwist last1_cmd_;
  CommandTwist last0_cmd_;
  
  int reverse_fl=1;
  int reverse_fr=1;
  int reverse_rl=1;
  int reverse_rr=1;

  int MOTION_MODE;

  //double publish_period;

  const int velocity_rolling_window_size =10.0;

  bool open_loop_;

  double cmd_vel_timeout_ =1.0; //seconds

  double reference_timeout ;

  std::string base_frame_id_;

  std::string odom_frame_id_;

  bool enable_odom_pub_=true;

  bool enable_odom_tf_=true;

};
}
#endif  
