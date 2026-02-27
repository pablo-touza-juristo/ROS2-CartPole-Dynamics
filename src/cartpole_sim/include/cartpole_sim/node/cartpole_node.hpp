#ifndef CARTPOLE_NODE_HPP
#define CARTPOLE_NODE_HPP

#include <Eigen/Dense>
#include <cartpole_sim/dynamics/cartpole.hpp>
#include <cartpole_sim/math/rk4_integrator.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace cartpole_sim::node {

class CartPoleNode : public rclcpp::Node
{
  public:
    CartPoleNode(dynamics::CartPole cartpole,
                 math::RK4Integrator rk4_integrator,
                 const Eigen::Vector4d& state);

  private:
    dynamics::CartPole cartpole_;
    math::RK4Integrator rk4_integrator_;
    Eigen::Vector4d state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_;
};

}  // namespace cartpole_sim::node

#endif
