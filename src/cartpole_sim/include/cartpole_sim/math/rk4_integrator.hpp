#ifndef RK4_INTEGRATION_HPP
#define RK4_INTEGRATION_HPP

#include <Eigen/Dense>
#include <functional>

namespace cartpole_sim::math {
class RK4Integrator
{
  public:
    RK4Integrator(double dt);

    /**
      This function takes on the role of calculating the next state of the
      system, recieving a reference to the actual state of the system and for
      reusability porpuses it recieves a function that calculates the dynamics
      of a generic system that has a state vector 4x1
    */
    Eigen::Vector4d numeric_integration(
        const Eigen::Vector4d& state,
        std::function<Eigen::Vector4d(const Eigen::Vector4d&)> compute_dynamics)
        const;

  private:
    double dt_;  // The step for the simulation
};
}  // namespace cartpole_sim::math
#endif
