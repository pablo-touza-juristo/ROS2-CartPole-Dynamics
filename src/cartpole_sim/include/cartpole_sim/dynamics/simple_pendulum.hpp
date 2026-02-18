#ifndef SIMPLE_PENDULUM_HPP
#define SIMPLE_PENDULUM_HPP

#include <math.h>

#include <Eigen/Dense>

namespace cartpole_sim::dynamics {

class SimplePendulum
{
  public:
    SimplePendulum(double gravity, double cable_longitude, double mass);
    double compute_energy(const Eigen::Vector2d& state) const;
    Eigen::Vector2d compute_dynamics(const Eigen::Vector2d& state) const;

  private:
    double gravity_;
    double cable_longitude_;
    double mass_;
};

}  // namespace cartpole_sim::dynamics

#endif
