#include <cartpole_sim/dynamics/cartpole.hpp>
#include <cartpole_sim/math/rk4_integrator.hpp>
#include <cartpole_sim/node/cartpole_node.hpp>
#include <memory>

#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
using cartpole_sim::dynamics::CartPole;
using cartpole_sim::math::RK4Integrator;
using cartpole_sim::node::CartPoleNode;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  RK4Integrator rk4_integrator = RK4Integrator(0.1);
  CartPole cartpole = CartPole(0.1, 1, 9.81, 1);
  Eigen::Vector4d initial_state;

  initial_state << 0, 0.1, 0, 0;

  std::shared_ptr<CartPoleNode> cartpole_node =
      std::make_shared<CartPoleNode>(cartpole, rk4_integrator, initial_state);

  rclcpp::spin(cartpole_node);
  rclcpp::shutdown();

  return 0;
}
