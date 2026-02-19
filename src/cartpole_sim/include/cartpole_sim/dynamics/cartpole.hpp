#ifndef CARTPOLE_HPP
#define CARTPOLE_HPP

#include <Eigen/Dense>

namespace cartpole::dynamics {

class CartPole
{
  public:
    CartPole(double pendulum_mass, double cable_longitude, double gravity,
             double cart_mass);
    Eigen::Vector4d compute_dynamics(const Eigen::Vector4d& state) const;
    double compute_mechanical_energy(const Eigen::Vector4d& state) const;

  private:
    double pendulum_mass_;
    double cable_longitude_;
    double gravity_;
    double cart_mass_;
    double input_force_;
};

}  // namespace cartpole::dynamics
#endif
