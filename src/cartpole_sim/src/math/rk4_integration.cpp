#include "cartpole_sim/math/rk4_integration.hpp"

namespace cartpole_sim::math {

RK4Integration::RK4Integration(double dt) : dt_(dt) {}

// Implementation of a Runge-Kutta fourth order numeric integrator
// for simulating the dynamics of a generic system whose state can
// be represented by a 4x1 state vector.
//
// I calculate the dynamics in the instant of the simulation, the state
// a half step ahead, another half step ahead and then a whole step. Then
// i calculate de resulting state of the system using the weighted sum
// of the four states
Eigen::Vector4d RK4Integration::numeric_integration(
    const Eigen::Vector4d& state,
    std::function<Eigen::Vector4d(Eigen::Vector4d)> compute_dynamics) const
{
  const auto k1 = compute_dynamics(state);
  const auto k2 = compute_dynamics(state + (0.5 * dt_ * k1));
  const auto k3 = compute_dynamics(state + (0.5 * dt_ * k2));
  const auto k4 = compute_dynamics(state + (dt_ * k3));

  const auto result = state + (dt_ / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

  return (result);
}

}  // namespace cartpole_sim::math
