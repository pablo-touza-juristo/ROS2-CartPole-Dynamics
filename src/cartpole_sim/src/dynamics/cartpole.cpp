#include "cartpole_sim/dynamics/cartpole.hpp"

#include <Eigen/Core>
#include <cmath>

namespace cartpole_sim::dynamics {

// Constructor for the CartPole class, i initialize the parameters using
// initialization lists and initialize the input_force_ paremeter as 0 so the
// cart starts in an still position
CartPole::CartPole(double pendulum_mass, double cable_longitude, double gravity,
                   double cart_mass)
    : pendulum_mass_(pendulum_mass),
      cable_longitude_(cable_longitude),
      gravity_(gravity),
      cart_mass_(cart_mass)
{
  input_force_ = 0;
}

// This function will compute the dynamics of the full system using the
// Lagrangian and resulting in the following formula: M(q)*q_dotdot + C(q,
// q_dot)*q_dot = t_g(q) + B*u
//
// Using the previous equation i can solve by solving for q_dotdot, solving
// for this vector leaves the formula as a linear system equation (Ax = b)
// being A the 2x2 matrix M(q), x being a 2x1 vector q_dotdot and b being the
// following operation: t_g(q) + B*u - C(q, q_dot)*q_dot
//
// Being the masses matrix a simetrical 2x2 matrix seems appropriate to use the
// ldlt resolution method for it's simplicity and speed.
Eigen::Vector4d CartPole::compute_dynamics(const Eigen::Vector4d& state) const
{
  // 2x2 Matrices mass and coriolis
  Eigen::Matrix<double, 2, 2> mass, coriolis;
  Eigen::Vector2d tau_gravity, control_vector,
      speed;  // the 2x1 vectors that will be used

  // Initialization of the matrices and vectors using the members of the
  // CartPole, and the actual state of the cartpole
  mass << (cart_mass_ + pendulum_mass_),
      (pendulum_mass_ * cable_longitude_ * cos(state(1))),
      (pendulum_mass_ * cable_longitude_ * cos(state(1))),
      (pendulum_mass_ * pow(cable_longitude_, 2));

  coriolis << 0,
      (-pendulum_mass_ * cable_longitude_ * state(3) * sin(state(1))), 0, 0;
  tau_gravity << 0,
      (-pendulum_mass_ * gravity_ * cable_longitude_ * sin(state(1)));
  control_vector << 1, 0;
  speed << state(2), state(3);

  // The resolution of the system yields a 2x1 vector that contains the
  // accelerations of the system in the actual state
  auto acceleration = mass.ldlt().solve(
      tau_gravity + control_vector * input_force_ - coriolis * speed);

  // Having calculated the accelerations of the system i combine them
  // with the velocities in the state vector to build the dot_state vector, it
  // being a 4x1 vector that contains all the information
  Eigen::Vector4d dot_state;
  dot_state << state(2), state(3), acceleration(0), acceleration(1);

  return dot_state;
}

// This function takes on the role of calculating the mechanical energy of the
// system it does not have an utility outside of corroborating the correctness
// of the simulation
double CartPole::compute_mechanical_energy(const Eigen::Vector4d& state) const
{
  double kinetic_energy, potential_energy, mechanical_energy, angular_position,
      linear_speed, angular_speed;

  angular_position = state(1);
  linear_speed = state(2);
  angular_speed = state(3);

  // The kintetical energy of the system, because there are two components (cart
  // and pendulum) the formula is the result of the sum of the two speeds
  // squared
  kinetic_energy =
      0.5 * (cart_mass_ + pendulum_mass_) * (linear_speed * linear_speed) +
      (pendulum_mass_ * linear_speed * angular_speed * cable_longitude_ *
       cos(angular_position)) +
      (0.5 * pendulum_mass_ * (cable_longitude_ * cable_longitude_) *
       (angular_speed * angular_speed));

  // The pendulum is the only one whose potential energy is important and
  // affects the system, so i only calculate the potential energy for the
  // pendulum
  potential_energy =
      -pendulum_mass_ * gravity_ * cable_longitude_ * cos(angular_position);

  mechanical_energy = kinetic_energy + potential_energy;

  return mechanical_energy;
}

}  // namespace cartpole_sim::dynamics
