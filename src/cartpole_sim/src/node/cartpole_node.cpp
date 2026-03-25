#include "cartpole_sim/node/cartpole_node.hpp"

#include <functional>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <string>

#include "cartpole_sim/constants.hpp"
#include "cartpole_sim/dynamics/cartpole.hpp"
#include "cartpole_sim/math/rk4_integrator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

namespace cartpole_sim::node {

CartPoleNode::CartPoleNode(bool intra_process_comms)
    : rclcpp_lifecycle::LifecycleNode(
          kCartPolePhysicsNodeName,
          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  this->declare_parameters(std::string(kNamespaceParamName),
                           get_default_cartpole_config());
  this->declare_parameters(std::string(kNamespaceParamName),
                           get_default_rk4_integrator_config());
  this->declare_parameters(std::string(kNamespaceParamName),
                           get_initial_state());
  cartpole_ = std::nullopt;
  rk4_integrator_ = std::nullopt;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartPoleNode::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "Node %s is being configured",
              this->get_name());
  /* Simulation objects configuration */
  cartpole_config_.pendulum_mass =
      this->get_parameter(
              get_full_param_name(std::string(kPendulumMassParamName)))
          .as_double();
  cartpole_config_.cart_mass =
      this->get_parameter(get_full_param_name(std::string(kCartMassParamName)))
          .as_double();
  cartpole_config_.cable_longitude =
      this->get_parameter(
              get_full_param_name(std::string(kCableLongitudeParamName)))
          .as_double();
  cartpole_config_.input_force =
      this->get_parameter(
              get_full_param_name(std::string(kInputForceParamName)))
          .as_double();
  cartpole_config_.gravity =
      this->get_parameter(get_full_param_name(std::string(kGravityParamName)))
          .as_double();

  rk4_integrator_config_.dt =
      this->get_parameter(get_full_param_name(std::string(kDtParamName)))
          .as_double();

  initial_state_ << this->get_parameter(get_full_param_name(std::string(
                                            kCartInitialPosParamName)))
                        .as_double(),
      this->get_parameter(
              get_full_param_name(std::string(kPendulumInitialPosParamName)))
          .as_double(),
      this->get_parameter(get_full_param_name(
                              std::string(kCartInitialLinearSpeedParamName)))
          .as_double(),
      this->get_parameter(get_full_param_name(std::string(
                              kPendulumInitialAngularSpeedParamName)))
          .as_double();

  state_ << initial_state_(0), initial_state_(1), initial_state_(2),
      initial_state_(3);

  /* ROS 2 Utilities configuration (Publisher, JointState msg and timer) */
  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      kJointStateTopicName, 10);

  // Correct size of the vectors of the message for the two joints
  // (cart and pendulum) that need updating
  joint_state_.name.resize(2);
  joint_state_.position.resize(2);
  joint_state_.velocity.resize(2);
  joint_state_.effort.resize(2);

  joint_state_.name[0] = kCartJointName;
  joint_state_.name[1] = kPendulumJointName;

  if (!cartpole_.has_value()) cartpole_.emplace(cartpole_config_);

  if (!rk4_integrator_.has_value())
    rk4_integrator_.emplace(rk4_integrator_config_);

  auto compute_dynamics =
      [this](const Eigen::Vector4d& state) -> Eigen::Vector4d
  { return this->cartpole_->compute_dynamics(state); };

  auto timer_callback = [this, compute_dynamics]() -> void
  {
    this->state_ =
        this->rk4_integrator_->numeric_integration(state_, compute_dynamics);

    joint_state_.header.stamp = this->get_clock()->now();
    // The state vector of the cartpole system is as follows
    // [x, theta, x_dot, theta_dot]
    joint_state_.position[0] = state_(0);
    joint_state_.position[1] = state_(1);

    joint_state_.velocity[0] = state_(2);
    joint_state_.velocity[1] = state_(3);

    joint_state_.effort[0] = cartpole_->get_input_force();
    joint_state_.effort[1] = 0;

    publisher_->publish(joint_state_);
  };

  // The state of the simulation will be updated at a 100Hz (every 10ms)
  timer_ = this->create_wall_timer(10ms, timer_callback);

  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Node %s has been configured",
              this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartPoleNode::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(this->get_logger(), "Node %s is being activated",
              this->get_name());

  rclcpp_lifecycle::LifecycleNode::on_activate(state);

  if (timer_->is_canceled()) timer_->reset();

  publisher_->on_activate();

  RCLCPP_INFO(this->get_logger(), "Node %s has been activated",
              this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartPoleNode::on_deactivate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(this->get_logger(), "Node %s is being deactivated",
              this->get_name());
  rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

  timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Node %s has been deactivated",
              this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartPoleNode::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(this->get_logger(), "Node %s is being has begun cleanup",
              this->get_name());
  timer_.reset();
  publisher_.reset();
  cartpole_.reset();
  rk4_integrator_.reset();

  RCLCPP_INFO(this->get_logger(), "Node %s has finished cleanup",
              this->get_name());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartPoleNode::on_shutdown(const rclcpp_lifecycle::State& state)
{
  rclcpp_lifecycle::LifecycleNode::on_shutdown(state);

  RCLCPP_INFO(this->get_logger(), "Node %s shutting down...", this->get_name());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

}  // namespace cartpole_sim::node
