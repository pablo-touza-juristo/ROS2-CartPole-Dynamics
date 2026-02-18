#include "cartpole_sim/math/rk4_integration.hpp"

#include "cartpole_sim/dynamics/simple_pendulum.hpp"

using cartpole_sim::dynamics::SimplePendulum;

namespace cartpole_sim::math {

RK4Integration::RK4Integration(SimplePendulum& simple_pendulum, double dt)
    : simple_pendulum_(&simple_pendulum), dt_(dt)
{}

Eigen::Vector2d RK4Integration::numeric_integration(
    const Eigen::Vector2d& state) const
{
  const auto k1 = simple_pendulum_->compute_dynamics(state);
  const auto k2 = simple_pendulum_->compute_dynamics(state + (0.5 * dt_ * k1));
  const auto k3 = simple_pendulum_->compute_dynamics(state + (0.5 * dt_ * k2));
  const auto k4 = simple_pendulum_->compute_dynamics(state + (dt_ * k3));

  const auto result = state + (dt_ / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

  return (result);
}

}  // namespace cartpole_sim::math
