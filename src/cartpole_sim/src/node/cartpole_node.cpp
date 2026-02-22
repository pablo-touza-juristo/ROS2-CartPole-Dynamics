#include "cartpole_sim/node/cartpole_node.hpp"

#include "cartpole_sim/constants.hpp"
#include "cartpole_sim/dynamics/cartpole.hpp"
#include "cartpole_sim/math/rk4_integrator.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using cartpole_sim::dynamics::CartPole;
using cartpole_sim::math::RK4Integrator;
using namespace std::chrono_literals;

namespace cartpole_sim::node {

CartPoleNode::CartPoleNode(CartPole& cartpole, RK4Integrator& rk4_integrator,
                           Eigen::Vector4d& state)
    : Node(kCartPolePhysicsNodeName),
      cartpole_(&cartpole),
      rk4_integrator_(&rk4_integrator),
      state_(state)
{
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

  auto timer_callback = [this]() -> void
  {
    auto compute_dynamics =
        [this](const Eigen::Vector4d& state) -> Eigen::Vector4d
    { return this->cartpole_->compute_dynamics(state); };

    this->state_ =
        this->rk4_integrator_->numeric_integration(state_, compute_dynamics);

    joint_state_.header.stamp = this->get_clock()->now();
    // The state vector of the cartpole system is as follows
    // [x, theta, x_dot, theta_dot]
    joint_state_.position[0] = state_(0);
    joint_state_.position[1] = state_(1);

    joint_state_.velocity[0] = state_(2);
    joint_state_.velocity[1] = state_(3);

    joint_state_.effort[0] = cartpole_->get_input_force_();
    joint_state_.effort[1] = 0;

    publisher_->publish(joint_state_);
  };

  // The state of the simulation will be updated at a 100Hz (every 10ms)
  timer_ = this->create_wall_timer(10ms, timer_callback);
}
}  // namespace cartpole_sim::node
