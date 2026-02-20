#ifndef RK4_INTEGRATION_HPP
#define RK4_INTEGRATION_HPP

#include <Eigen/Dense>
#include <functional>

namespace cartpole_sim::math {
class RK4Integrator
{
  public:
    RK4Integrator(double dt);
    Eigen::Vector4d numeric_integration(
        const Eigen::Vector4d& state,
        std::function<Eigen::Vector4d(Eigen::Vector4d)> compute_dynamics) const;

  private:
    double dt_;  // The step for the simulation
};
}  // namespace cartpole_sim::math
#endif
