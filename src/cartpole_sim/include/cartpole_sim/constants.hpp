#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <map>
#include <rclcpp/parameter_value.hpp>
#include <string>
#include <string_view>

namespace cartpole_sim::node {
/* Basic static names for the topics and joints of the system */
inline constexpr char kCartPolePhysicsNodeName[] = "cartpole_physics_node";
inline constexpr char kJointStateTopicName[] = "joint_states";
inline constexpr char kCartJointName[] = "cart_joint";
inline constexpr char kPendulumJointName[] = "pendulum_joint";

/* Constant for the name of the namespace */
constexpr std::string_view kNamespaceParamName = "cartpole_sim";

/* Names for the parameters of the CartPole members */
constexpr std::string_view kPendulumMassParamName = "pendulum_mass";
constexpr std::string_view kCableLongitudeParamName = "cable_longitude";
constexpr std::string_view kGravityParamName = "gravity";
constexpr std::string_view kCartMassParamName = "cart_mass";
constexpr std::string_view kInputForceParamName = "input_force";

/* Names for the parameters of the RK4Integrator members */
constexpr std::string_view kDtParamName = "dt";

inline std::map<std::string, rclcpp::ParameterValue>
get_default_cartpole_config()
{
  return {{std::string(kPendulumMassParamName), rclcpp::ParameterValue(1.0)},
          {std::string(kCableLongitudeParamName), rclcpp::ParameterValue(1.0)},
          {std::string(kGravityParamName), rclcpp::ParameterValue(9.81)},
          {std::string(kCartMassParamName), rclcpp::ParameterValue(1.0)},
          {std::string(kInputForceParamName), rclcpp::ParameterValue(0.0)}};
}

inline std::map<std::string, rclcpp::ParameterValue>
get_default_rk4_integrator_config()
{
  return {{std::string(kDtParamName), rclcpp::ParameterValue(0.01)}};
}

inline std::string get_full_param_name(const std::string& param_name)
{
  return std::string("cartpole_sim." + param_name);
}

}  // namespace cartpole_sim::node

#endif
