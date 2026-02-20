#include <iostream>

#include "cartpole_sim/dynamics/cartpole.hpp"
#include "cartpole_sim/math/rk4_integration.hpp"

using cartpole_sim::dynamics::CartPole;
using cartpole_sim::math::RK4Integration;

int main()
{
  double t_sim = 0.0;
  const double dt = 0.001, t_max = 10;

  CartPole cartpole = CartPole(0.1, 1.0, 9.81, 0.1);
  RK4Integration rk4_integrator = RK4Integration(cartpole, dt);
  Eigen::Vector4d state;

  state(0) = 0;
  state(1) = 0.1;
  state(2) = 0;
  state(3) = 0;

  std::cout << "time,x,theta,x_dot,theta_dot,mechanical_energy" << std::endl;

  while (t_sim < t_max) {
    state = rk4_integrator.numeric_integration(state);
    std::cout << t_sim << "," << state(0) << "," << state(1) << "," << state(2)
              << "," << state(3) << ","
              << cartpole.compute_mechanical_energy(state) << std::endl;

    t_sim += dt;
  }

  return (0);
}
