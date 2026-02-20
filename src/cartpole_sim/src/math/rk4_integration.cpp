#include "cartpole_sim/math/rk4_integration.hpp"

#include "cartpole_sim/dynamics/cartpole.hpp"

// For better code legibility
using cartpole_sim::dynamics::CartPole;

namespace cartpole_sim::math {

RK4Integration::RK4Integration(CartPole& cartpole, double dt)
    : cartpole_(&cartpole), dt_(dt)
{}

// Implementation of a Runge-Kutta fourth order numeric integrator
// for simulating the dynamics of the cartpole system.
//
// I calculate the dynamics in the instant of the simulation, the state
// a half step ahead, another half step ahead and then a whole step. Then
// i calculate de resulting state of the system using the weighted sum
// of the four states
Eigen::Vector4d RK4Integration::numeric_integration(
    const Eigen::Vector4d& state) const
{
  const auto k1 = cartpole_->compute_dynamics(state);
  const auto k2 = cartpole_->compute_dynamics(state + (0.5 * dt_ * k1));
  const auto k3 = cartpole_->compute_dynamics(state + (0.5 * dt_ * k2));
  const auto k4 = cartpole_->compute_dynamics(state + (dt_ * k3));

  const auto result = state + (dt_ / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

  return (result);
}

}  // namespace cartpole_sim::math
