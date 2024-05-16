#ifndef FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_H_
#define FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_H_


#pragma once
#include <tuple>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "rcppmath/rolling_mean_accumulator.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "boost/numeric/odeint.hpp"

// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/point.hpp>
//#include <gazebo_msgs/gazebo_msgs/srv/get_entity_state.hpp>


namespace four_wheel_steering_controller
{

template <typename Model>
class MotionModel {
 public:
  using State = typename Model::state_type;
  using Command = typename Model::control_type;

 public:
  State StepForward(State x0, Command u, double t0, double tf, double dt) {
    State x = x0;
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<typename Model::state_type>(),
        Model(u), x, t0, tf, dt);
    return x;
  }
};

class DualAckermanModel {
 public:
  using state_type = std::vector<double>;

  struct control_type 
  {
    double v;
    double phi;
  };

 public:
  DualAckermanModel(double L, control_type u) : L_(L), u_(u){};

  // x1 = x, x2 = y, x3 = theta
  void operator()(const state_type& x, state_type& xd, double) 
  {
    xd[0] = u_.v * std::cos(u_.phi) * std::cos(x[2]);
    xd[1] = u_.v * std::cos(u_.phi) * std::sin(x[2]);
    xd[2] = 2 * u_.v * std::sin(u_.phi) / L_;
  }

 private:
  double L_;
  control_type u_;
};

class ParallelModel 
{
 public:
  using state_type = std::vector<double>;

  struct control_type 
  {
    double v;
    double phi;
  };

 public:
  ParallelModel(control_type u) : u_(u){};

  // x1 = x, x2 = y, x3 = theta
  void operator()(const state_type& x, state_type& xd, double) 
  {
    xd[0] = u_.v * std::cos(x[2] + u_.phi);
    xd[1] = u_.v * std::sin(x[2] + u_.phi);
    xd[2] = 0;
  }

 private:
  control_type u_;
};

class SpinningModel 
{
 public:
  using state_type = std::vector<double>;

  struct control_type 
  {
    double w;
  };

 public:
  SpinningModel(control_type u) : u_(u){};

  // x1 = x, x2 = y, x3 = theta
  void operator()(const state_type& x, state_type& xd, double) {
    xd[0] = 0;
    xd[1] = 0;
    xd[2] = u_.w;
  }

 private:
  control_type u_;
};
 
class Odometry
{
  public:
  
  double getX() { return position_x_; }

  double getY() { return position_y_; }

  geometry_msgs::msg::Quaternion get_orientation() { return odom_quat; }

  double ConvertInnerAngleToCentral(double angle);

  void UpdateOdometry(int motion_mode, double linear, double angular, double angle,double sign_ang, double dt);

  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  geometry_msgs::msg::Quaternion odom_quat;

  private:

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
 
  const double wheelbase =0.46; //0.492; //  // needs to figure out
  const double track = 0.412; //0.39; //.364;                     // needs to figure out
 
  };
}

#endif 