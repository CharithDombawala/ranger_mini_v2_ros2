#include <urdf_geometry_parser/urdf_geometry_parser.h>
#include <cmath>

namespace urdf_geometry_parser{

  /*
   * \brief Check if the link is modeled as a cylinder
   * \param link Link
   * \return true if the link is modeled as a Cylinder; false otherwise
   */
  static bool isCylinder(urdf::LinkConstSharedPtr& link)
  {
    if (!link)
    {
      RCLCPP_ERROR(rclcpp::get_logger("urdf_geometry_parser"), "Link == NULL.");     
      return false;
    }

    if (!link->collision)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("urdf_geometry_parser"), "Link " << link->name << " does not have collision description. Add collision description for link to urdf.");      
      return false;
    }

    if (!link->collision->geometry)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("urdf_geometry_parser"), "Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
      return false;
    }

    if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("urdf_geometry_parser"), "Link " << link->name << " does not have cylinder geometry");      
      return false;
    }

    return true;
  }

  /*
   * \brief Get the wheel radius
   * \param [in]  wheel_link   Wheel link
   * \param [out] wheel_radius Wheel radius [m]
   * \return true if the wheel radius was found; false otherwise
   */
  static bool getWheelRadius(urdf::LinkConstSharedPtr wheel_link, double& wheel_radius)
  {
    if (!isCylinder(wheel_link))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("urdf_geometry_parser"), "Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
      return false;
    }

    wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
    return true;
  }


  UrdfGeometryParser::UrdfGeometryParser(rclcpp::Node& root_nh, const std::string& base_link):
    base_link_(base_link)
  {
    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.has_parameter(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.get_parameter(model_param_name,robot_model_str))
    {
        RCLCPP_ERROR(rclcpp::get_logger("urdf_geometry_parser"), "Robot description couldn't be retrieved from parameter server.");    
    }
    else
    {
      model_ = urdf::parseURDF(robot_model_str);
      if(!model_)
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("urdf_geometry_parser"), "Could not parse the urdf robot model " << model_param_name);
    }
  }

  bool UrdfGeometryParser::getTransformVector(const std::string& joint_name, const std::string& parent_link_name
                                                , urdf::Vector3 &transform_vector)
  {
    if(model_)
    {
      urdf::JointConstSharedPtr joint(model_->getJoint(joint_name));
      if (!joint)
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("urdf_geometry_parser"),joint_name << " couldn't be retrieved from model description");
        return false;
      }

      transform_vector = joint->parent_to_joint_origin_transform.position;
      while(joint->parent_link_name != parent_link_name)
      {
        urdf::LinkConstSharedPtr link_parent(model_->getLink(joint->parent_link_name));
        if (!link_parent || !link_parent->parent_joint)
        {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("urdf_geometry_parser"),joint->parent_link_name<< " couldn't be retrieved from model description or his parent joint");
          return false;
        }
        joint = link_parent->parent_joint;
        transform_vector = transform_vector + joint->parent_to_joint_origin_transform.position;
      }
      return true;
    }
    else
      return false;
  }

  bool UrdfGeometryParser::getDistanceBetweenJoints(const std::string& first_joint_name,
                                                      const std::string& second_joint_name,
                                                      double& distance)
  {
    urdf::Vector3 first_transform;
    if(!getTransformVector(first_joint_name, base_link_, first_transform))
      return false;

    urdf::Vector3 second_transform;
    if(!getTransformVector(second_joint_name, base_link_, second_transform))
      return false;

    // Calculate the Euclidean distance using the Pythagorean formula
    distance = std::sqrt(std::pow(first_transform.x - second_transform.x, 2)
                         + std::pow(first_transform.y - second_transform.y, 2));
    return true;
  }

  bool UrdfGeometryParser::getJointRadius(const std::string& joint_name,
                                            double& radius)
  {
    if(model_)
    {
      urdf::JointConstSharedPtr joint(model_->getJoint(joint_name));
      // Get wheel radius
      if (!getWheelRadius(model_->getLink(joint->child_link_name), radius))
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("urdf_geometry_parser"),"Couldn't retrieve " << joint_name << " wheel radius");
        return false;
      }
      return true;
    }
    else
      return false;
  }

  bool UrdfGeometryParser::getJointSteeringLimits(const std::string& joint_name,
                              double& steering_limit)
  {
    if(model_)
    {
      urdf::JointConstSharedPtr joint(model_->getJoint(joint_name));
      if(joint->type == urdf::Joint::REVOLUTE)
      {
        const double lower_steering_limit = fabs(joint->limits->lower);
        const double upper_steering_limit = fabs(joint->limits->upper);
        if(lower_steering_limit > upper_steering_limit)
          steering_limit = upper_steering_limit;
        else
          steering_limit = lower_steering_limit;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("urdf_geometry_parser"),"Joint "<<joint_name<<" steering limit is "<<steering_limit*180.0/M_PI<<" in degrees");
        return true;
      }
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("urdf_geometry_parser"),"Couldn't get joint "<<joint_name<<" steering limit, is it of type REVOLUTE ?");
    }
    return false;
  }
}
