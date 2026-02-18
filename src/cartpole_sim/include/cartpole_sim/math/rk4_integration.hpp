#ifndef RK4_INTEGRATION_HPP
#define RK4_INTEGRATION_HPP

#include <Eigen/Dense>

namespace cartpole_sim::dynamics {
class SimplePendulum;
}

namespace cartpole_sim::math {
class RK4Integration
{
  public:
    RK4Integration(cartpole_sim::dynamics::SimplePendulum& simple_pendulum,
                   double dt);
    Eigen::Vector2d numeric_integration(const Eigen::Vector2d& state) const;

  private:
    cartpole_sim::dynamics::SimplePendulum*
        simple_pendulum_;  // The pendulum i am going to simulate
    double dt_;            // The step for the simulation
};
}  // namespace cartpole_sim::math
#endif
