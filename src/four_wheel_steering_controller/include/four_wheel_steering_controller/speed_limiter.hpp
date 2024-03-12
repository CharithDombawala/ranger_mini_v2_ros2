#pragma once
namespace four_wheel_steering_controller
{
  template<typename T>
  T clamp(T x, T min, T max)
  {
    return std::min(std::max(min, x), max);
  }
  class SpeedLimiter
  {
  public:
    SpeedLimiter(
      bool has_velocity_limits = false,
      bool has_acceleration_limits = false,
      bool has_jerk_limits = false,
      double min_velocity = 0.0,
      double max_velocity = 0.0,
      double min_acceleration = 0.0,
      double max_acceleration = 0.0,
      double min_jerk = 0.0,
      double max_jerk = 0.0
    );

    double limit_lin(double& v, double v0, double v1, double dt);

    double limit_ang(double& v, double v0, double v1, double dt);

    double limit_velocity(double& v);

   
    double limit_acceleration(double& v, double v0, double dt);

 
    double limit_jerk(double& v, double v0, double v1, double dt);

  public:
    bool has_velocity_limits;
    bool has_acceleration_limits;
    bool has_jerk_limits;

    // Velocity limits:
    double min_velocity;
    double max_velocity;

    // Acceleration limits:
    double min_acceleration;
    double max_acceleration;

    // Jerk limits:
    double min_jerk;
    double max_jerk;
  };

} 
