#include "cartpole_sim/dynamics/simple_pendulum.hpp"

#include <cmath>

namespace cartpole_sim::dynamics {

// Constructor for a SimplePendulum object, i initilialize all the variables
// using it's own constructor for defined behaviour
SimplePendulum::SimplePendulum(double gravity, double cable_longitude,
                               double mass)
    : gravity_(gravity), cable_longitude_(cable_longitude), mass_(mass)
{}

// Function for dynamics computation of the Simple Pendulum, it takes an state
// column vector that contains theta and theta_dot, which i use to calculate
// dot_state.
//
// The return of this function is dot_state, a column vector of 1x2 that
// contains thetha_dot (angular velocity) and theta_dot_dot (angular
// acceleration), using the langrangian to calculate the later
Eigen::Vector2d SimplePendulum::compute_dynamics(
    const Eigen::Vector2d& state) const
{
  Eigen::Vector2d dot_state;

  dot_state(0) = state(1);
  dot_state(1) = -(gravity_ * sin(state(0)) / (cable_longitude_));

  return (dot_state);
}

double SimplePendulum::compute_energy(const Eigen::Vector2d& state) const
{
  double theta = state(0, 0);      // Recuerda: state(fila, col)
  double theta_dot = state(1, 0);  // Y el índice es 0 para la columna

  // Energía Cinética
  double v = cable_longitude_ * theta_dot;
  double kinetic = 0.5 * mass_ * v * v;

  // Energía Potencial (referencia en el punto más bajo)
  double height = cable_longitude_ * (1.0 - std::cos(theta));
  double potential = mass_ * gravity_ * height;

  return kinetic + potential;
}

}  // namespace cartpole_sim::dynamics
