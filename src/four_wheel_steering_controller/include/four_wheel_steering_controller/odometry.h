#ifndef FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_H_
#define FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_H_


#pragma once
#include <tuple>
#include <vector>
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "rcppmath/rolling_mean_accumulator.hpp"


namespace Odometry
{
  
class Odometry
{
public:


    Odometry(size_t velocity_rolling_window_size = 10);


    void init(const rclcpp::Time & time);

  
    bool update(const double& fl_speed, const double& fr_speed, const double& rl_speed, const double& rr_speed,
                double front_steering, double rear_steering, const rclcpp::Time &time,bool lin_y=false);


    double getHeading() const
    {
      return heading_;
    }

    double getX() const
    {
      return x_;
    }


    double getY() const
    {
      return y_;
    }


    double getLinear() const
    {
      return linear_;
    }


    double getLinearX() const
    {
      return linear_x_;
    }


    double getLinearY() const
    {
      return linear_y_;
    }

    double getAngular() const
    {
      return angular_;
    }


    bool setWheelParams(double steering_track, double wheel_steering_y_offset, double wheel_radius, double wheel_base);

  
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

    void integrateXY(double linear_x, double linear_y, double angular);

    void integrateRungeKutta2(double linear, double angular);


    void integrateExact(double linear, double angular);

    void resetAccumulators();

    /// Current timestamp:
    rclcpp::Time last_update_timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_, linear_x_, linear_y_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double steering_track_;
    double wheel_steering_y_offset_;
    double wheel_radius_;
    double wheel_base_;

    /// Previous wheel position/state [rad]:
    double wheel_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    rcppmath::RollingMeanAccumulator<double> linear_accel_acc_;
    rcppmath::RollingMeanAccumulator<double> linear_jerk_acc_;
    rcppmath::RollingMeanAccumulator<double> front_steer_vel_acc_;
    rcppmath::RollingMeanAccumulator<double> rear_steer_vel_acc_;
    double linear_vel_prev_, linear_accel_prev_;
    double front_steer_vel_prev_, rear_steer_vel_prev_;
  };
}

#endif 