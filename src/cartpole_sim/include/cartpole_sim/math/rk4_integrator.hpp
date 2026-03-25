#ifndef RK4_INTEGRATION_HPP
#define RK4_INTEGRATION_HPP

#include <Eigen/Dense>
#include <functional>

namespace cartpole_sim::math {

struct Config
{
    double dt;
};

class RK4Integrator
{
  public:
    RK4Integrator(const Config& config);

    /**
      This function takes on the role of calculating the next state of the
      system, recieving a reference to the actual state of the system and for
      reusability porpuses it recieves a function that calculates the dynamics
      of a generic system that has a state vector 4x1
    */
    template <typename Callable>
    Eigen::Vector4d numeric_integration(const Eigen::Vector4d& state,
                                        Callable&& compute_dynamics) const;

  private:
    double dt_;  // The step for the simulation
};

// Implementation of a Runge-Kutta fourth order numeric integrator
// for simulating the dynamics of a generic system whose state can
// be represented by a 4x1 state vector.
//
// I calculate the dynamics in the instant of the simulation, the state
// a half step ahead, another half step ahead and then a whole step. Then
// i calculate de resulting state of the system using the weighted sum
// of the four states
template <typename Callable>
Eigen::Vector4d RK4Integrator::numeric_integration(
    const Eigen::Vector4d& state, Callable&& compute_dynamics) const
{
  const auto k1 = compute_dynamics(state);
  const auto k2 = compute_dynamics(state + (0.5 * dt_ * k1));
  const auto k3 = compute_dynamics(state + (0.5 * dt_ * k2));
  const auto k4 = compute_dynamics(state + (dt_ * k3));

  const auto result = state + (dt_ / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

  return (result);
}

}  // namespace cartpole_sim::math
#endif
