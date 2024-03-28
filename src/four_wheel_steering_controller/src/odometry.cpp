#include <four_wheel_steering_controller/odometry.h>
#include <cmath>
#include <iostream>
#include <boost/bind.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;


namespace four_wheel_steering_controller
{

void Odometry::UpdateOdometry(int MOTION_MODE, double linear, double angular,double angle,double sign_ang, double dt) 
{


  // update odometry calculations
  if (MOTION_MODE == 3) {
    DualAckermanModel::state_type x = {position_x_, position_y_, theta_};
    DualAckermanModel::control_type u;
    u.v = linear;
    u.phi = sign_ang*ConvertInnerAngleToCentral(angle);
    
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
        DualAckermanModel(wheelbase, u), x, 0.0, dt, (dt / 10.0));
   
    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } 
  else if (MOTION_MODE == 1)       
  {
    ParallelModel::state_type x = {position_x_, position_y_, theta_};
    ParallelModel::control_type u;
    u.v = linear;
    u.phi = angle;
    
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
        ParallelModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];

  } 
  else if (MOTION_MODE == 2) 
  {
    SpinningModel::state_type x = {position_x_, position_y_, theta_};
    SpinningModel::control_type u;
    u.w = angular;

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
        SpinningModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  }
  else
  {
    position_x_ = position_x_;
    position_y_ = position_y_;
    theta_ = theta_;
  }

  // update odometry topics
  odom_quat = createQuaternionMsgFromYaw(theta_);

  

}


double Odometry::ConvertInnerAngleToCentral(double angle)
{
  double phi = 0;
  double phi_i = std::abs(angle);


  phi = std::atan(wheelbase * std::sin(phi_i) /  // x Ackerman
                  (wheelbase * std::cos(phi_i) +
                   track * std::sin(phi_i)));


  phi *= angle >= 0 ? 1.0 : -1.0;
  return phi;
}

geometry_msgs::msg::Quaternion Odometry::createQuaternionMsgFromYaw(double yaw) 
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}


// std::tuple<double, double, double> Odometry::getPose(const std::string& robot_name, rclcpp::Node::SharedPtr nh) 
// {
//   // Create a service client for get_entity_state
//   auto client = nh->create_client<gazebo_msgs::srv::GetEntityState>("/gazebo/get_entity_state");

//   // Call the service and check for success
//   auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
//   request->name = robot_name;
//   request->reference_frame = "world";

//   auto future_result = client->async_send_request(request);
//   auto result = future_result.get();
//   if (result->success) 
//   {
//     auto pose = result->state.pose;
//     auto position = pose.position;
//     auto orientation = pose.orientation;

//     return std::make_tuple(position.x, position.y, orientation.w);
//   } 
//   else 
//   {
//     RCLCPP_ERROR(nh->get_logger(), "Failed to get robot pose from Gazebo");
//     return std::make_tuple(-1.0, -1.0, -1.0);
//   }

// }

} // namespace four_wheel_steering_controller
