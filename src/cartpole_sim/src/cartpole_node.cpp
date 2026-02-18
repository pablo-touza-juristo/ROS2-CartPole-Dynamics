#include <iostream>

#include "cartpole_sim/dynamics/simple_pendulum.hpp"
#include "cartpole_sim/math/rk4_integration.hpp"

using cartpole_sim::dynamics::SimplePendulum;
using cartpole_sim::math::RK4Integration;

int main()
{
  double t_sim = 0.0;
  const double dt = 0.001, t_max = 10;
  SimplePendulum simple_pendulum = SimplePendulum(9.81, 0.1, 1.0);
  RK4Integration rk4_integrator = RK4Integration(simple_pendulum, dt);
  Eigen::Vector2d state;

  state(0) = 0.1;
  state(1) = 0;

  std::cout << "time,theta,theta_dot,energy" << std::endl;

  while (t_sim < t_max) {
    state = rk4_integrator.numeric_integration(state);
    std::cout << t_sim << "," << state(0) << "," << state(1) << ","
              << simple_pendulum.compute_energy(state) << std::endl;

    t_sim += dt;
  }

  return (0);
}
