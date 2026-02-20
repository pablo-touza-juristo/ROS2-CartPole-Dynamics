#ifndef CARTPOLE_NODE_HPP
#define CARTPOLE_NODE_HPP

#include "rclcpp/node.hpp"

namespace cartpole_sim::dynamics {
class CartPole;
}

namespace cartpole_sim::math {
class RK4Integrator;
}

namespace cartpole_sim {

class CartPoleNode : rclcpp::Node
{
  public:
  private:
    Cartpole* cartpole_;
    RK4Integrator* rk4_integrator;
};

}  // namespace cartpole_sim

#endif
