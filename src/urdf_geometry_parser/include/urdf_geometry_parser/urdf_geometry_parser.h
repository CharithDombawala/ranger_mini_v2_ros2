#ifndef URDF_GEOMETRY_PARSER_URDF_GEOMETRY_PARSER_H
#define URDF_GEOMETRY_PARSER_URDF_GEOMETRY_PARSER_H

#include "rclcpp/rclcpp.hpp"

#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>

namespace urdf_geometry_parser {

class UrdfGeometryParser {

public:
  UrdfGeometryParser(rclcpp::Node& root_nh, const std::string &base_link);

  /**
   * \brief Get transform vector between the joint and parent_link
   * \param [in] joint_name   Child joint_name from where to begin
   * \param [in] parent_link_name Name of link to find as parent of the joint
   * \param [out] transform_vector Distance vector between joint and parent_link [m]
   * \return true if the transform vector was found; false otherwise
   */
  bool getTransformVector(const std::string& joint_name,
                          const std::string& parent_link_name,
                          urdf::Vector3& transform_vector);

  /**
   * \brief Get distance between two joints from the URDF
   * \param first_joint_name Name of the first joint
   * \param second_joint_name Name of the second joint
   * \param distance Distance in meter between first and second joint
   * \return true if the distance was found; false otherwise
   */
  bool getDistanceBetweenJoints(const std::string& first_joint_name,
                                const std::string& second_joint_name,
                                double& distance);

  /**
   * \brief Get joint radius from the URDF
   * \param joint_name Name of the joint
   * \param distance Distance in meter between first and second joint
   * \return true if the radius was found; false otherwise
   */
  bool getJointRadius(const std::string& joint_name,
                                double& radius);

  /**
   * \brief Get joint steering limit from the URDF
   *        considering the upper and lower limit is the same
   * \param joint_name Name of the joint
   * \param steering_limit [rad]
   * \return true if the steering limit was found; false otherwise
   */
  bool getJointSteeringLimits(const std::string& joint_name,
                              double& steering_limit);
private:
  std::string base_link_;

  urdf::ModelInterfaceSharedPtr model_;
};

} // urdf_geometry_parser

#endif
