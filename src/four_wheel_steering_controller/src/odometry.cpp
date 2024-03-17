#include </home/charith-2204/ranger_mini_v2/src/four_wheel_steering_controller/include/four_wheel_steering_controller/odometry.h>
#include <cmath>
#include <iostream>
#include <boost/bind.hpp>

namespace Odometry
{
  
  Odometry::Odometry(size_t velocity_rolling_window_size): last_update_timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , linear_x_(0.0)
  , linear_y_(0.0)
  , angular_(0.0)
  , steering_track_(0.0)
  , wheel_steering_y_offset_(0.0)
  , wheel_radius_(0.0)
  , wheel_base_(0.0)
  , wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_accel_acc_(velocity_rolling_window_size)
  , linear_jerk_acc_(velocity_rolling_window_size)
  , front_steer_vel_acc_(velocity_rolling_window_size)
  , rear_steer_vel_acc_(velocity_rolling_window_size)
  {
  }

  void Odometry::init(const rclcpp::Time & time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    last_update_timestamp_ = time;
  }

  bool Odometry::update(const double &fl_speed, const double &fr_speed,
                        const double &rl_speed, const double &rr_speed,
                        double front_steering, double rear_steering, const rclcpp::Time &time,bool lin_y)
  {

    //  if(lin_y==true) //swaping steering_track_ and wheel_base_
    // { 
    //    steering_track_   = 0.494;
    //    wheel_base_       = 0.364;
    // }
    const double front_tmp = cos(front_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
    const double front_left_tmp = front_tmp/sqrt(1-steering_track_*front_tmp*cos(front_steering)
                                               +pow(steering_track_*front_tmp/2,2));
    const double front_right_tmp = front_tmp/sqrt(1+steering_track_*front_tmp*cos(front_steering)
                                                +pow(steering_track_*front_tmp/2,2));
    const double fl_speed_tmp = fl_speed * (1/(1-wheel_steering_y_offset_*front_left_tmp));
    const double fr_speed_tmp = fr_speed * (1/(1-wheel_steering_y_offset_*front_right_tmp));
    const double front_linear_speed = wheel_radius_ * copysign(1.0, fl_speed_tmp+fr_speed_tmp)*
        sqrt((pow(fl_speed,2)+pow(fr_speed,2))/(2+pow(steering_track_*front_tmp,2)/2.0));

    const double rear_tmp = cos(rear_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
    const double rear_left_tmp = rear_tmp/sqrt(1-steering_track_*rear_tmp*cos(rear_steering)
                                               +pow(steering_track_*rear_tmp/2,2));
    const double rear_right_tmp = rear_tmp/sqrt(1+steering_track_*rear_tmp*cos(rear_steering)
                                                +pow(steering_track_*rear_tmp/2,2));
    const double rl_speed_tmp = rl_speed * (1/(1-wheel_steering_y_offset_*rear_left_tmp));
    const double rr_speed_tmp = rr_speed * (1/(1-wheel_steering_y_offset_*rear_right_tmp));
    const double rear_linear_speed = wheel_radius_ * copysign(1.0, rl_speed_tmp+rr_speed_tmp)*
        sqrt((pow(rl_speed_tmp,2)+pow(rr_speed_tmp,2))/(2+pow(steering_track_*rear_tmp,2)/2.0));

    angular_ = (front_linear_speed*front_tmp + rear_linear_speed*rear_tmp)/2.0;

    linear_x_ = (front_linear_speed*cos(front_steering) + rear_linear_speed*cos(rear_steering))/2.0;
    linear_y_ = (front_linear_speed*sin(front_steering) - wheel_base_*angular_/2.0
                + rear_linear_speed*sin(rear_steering) + wheel_base_*angular_/2.0)/2.0;
    // if(lin_y==true)
    // {
    // linear_y_ = (front_linear_speed*cos(front_steering) + rear_linear_speed*cos(rear_steering))/2.0;
    // linear_x_ = 0.0;
    // linear_x_ = (front_linear_speed*sin(front_steering) - wheel_base_*angular_/2.0
    //             + rear_linear_speed*sin(rear_steering) + wheel_base_*angular_/2.0)/2.0;
    // }
    // else
    // {
    // linear_x_ = (front_linear_speed*cos(front_steering) + rear_linear_speed*cos(rear_steering))/2.0;
    // linear_y_ = (front_linear_speed*sin(front_steering) - wheel_base_*angular_/2.0
    //             + rear_linear_speed*sin(rear_steering) + wheel_base_*angular_/2.0)/2.0;
    // }

    linear_ =  copysign(1.0, rear_linear_speed)*sqrt(pow(linear_x_,2)+pow(linear_y_,2));

    /// Compute x, y and heading using velocity

  
    const double dt = (time.seconds() - last_update_timestamp_.seconds());

    if (dt < 0.0001)
      return false; // Interval too small to integrate with

 
    last_update_timestamp_ =time;

    /// Integrate odometry:
    integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt);

    linear_accel_acc_.accumulate((linear_vel_prev_ - linear_)/dt);
    linear_vel_prev_ = linear_;
    linear_jerk_acc_.accumulate((linear_accel_prev_ - linear_accel_acc_.getRollingMean())/dt);
    linear_accel_prev_ = linear_accel_acc_.getRollingMean();
    front_steer_vel_acc_.accumulate((front_steer_vel_prev_ - front_steering)/dt);
    front_steer_vel_prev_ = front_steering;
    rear_steer_vel_acc_.accumulate((rear_steer_vel_prev_ - rear_steering)/dt);
    rear_steer_vel_prev_ = rear_steering;

    std::cout << "----values in odometry cpp------ \n"
          << " steering_track_: " << steering_track_ << std::endl
          << " wheel_steering_y_offset_: " << wheel_steering_y_offset_ << std::endl
          << " wheel_radius_: " << wheel_radius_ << std::endl
          << " wheel_base_: " << wheel_base_ << std::endl
          << " wheel_old_pos_: " << wheel_old_pos_ << std::endl;


    return true;
  }

  bool Odometry::setWheelParams(double steering_track, double wheel_steering_y_offset, double wheel_radius, double wheel_base)
  {
    steering_track_   = steering_track;
    wheel_steering_y_offset_ = wheel_steering_y_offset;
    wheel_radius_     = wheel_radius;
    wheel_base_       = wheel_base;

    return true;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateXY(double linear_x, double linear_y, double angular)
  {
    const double delta_x = linear_x*cos(heading_) - linear_y*sin(heading_);
    const double delta_y = linear_x*sin(heading_) + linear_y*cos(heading_);

    x_ += delta_x;
    y_ += delta_y;
    heading_ += angular;
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

  void Odometry::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
    }
  }

  void Odometry::resetAccumulators()
  {
    linear_accel_acc_ =  rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
    linear_jerk_acc_ =  rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
    front_steer_vel_acc_ =  rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
    rear_steer_vel_acc_ =  rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  }

} // namespace four_wheel_steering_controller
