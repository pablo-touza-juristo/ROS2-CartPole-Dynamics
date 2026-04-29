#include "cartpole_sim/math/rk4_integrator.hpp"

namespace cartpole_sim::math {

RK4Integrator::RK4Integrator(const Config& config) : dt_(config.dt) {}

}  // namespace cartpole_sim::math
