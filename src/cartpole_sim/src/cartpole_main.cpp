#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include "cartpole_sim/node/cartpole_node.hpp"

using cartpole_sim::node::CartPoleNode;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto cartpole_node = std::make_shared<CartPoleNode>();

  executor.add_node(cartpole_node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
