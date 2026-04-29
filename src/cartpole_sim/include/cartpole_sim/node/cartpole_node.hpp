#ifndef CARTPOLE_NODE_HPP
#define CARTPOLE_NODE_HPP

#include <Eigen/Dense>
#include <optional>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>

#include "cartpole_sim/dynamics/cartpole.hpp"
#include "cartpole_sim/math/rk4_integrator.hpp"

namespace cartpole_sim::node {

/*
 * This class serves as a container for the physics engine
 * so the ROS2 Middleware can be used, the physics will be calculated
 * via the timer_ member and the states of the system will published using
 * the publisher_ private member */
class CartPoleNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
    explicit CartPoleNode(bool intra_proccess_comms = false);

    /* Methods of the class */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&);

  private:
    /* Members of the class */
    dynamics::Config cartpole_config_;
    std::optional<dynamics::CartPole> cartpole_;
    math::Config rk4_integrator_config_;
    std::optional<math::RK4Integrator> rk4_integrator_;
    Eigen::Vector4d state_;
    Eigen::Vector4d initial_state_;
    rclcpp_lifecycle::LifecyclePublisher<
        sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_sim_service_;
};

}  // namespace cartpole_sim::node

#endif
