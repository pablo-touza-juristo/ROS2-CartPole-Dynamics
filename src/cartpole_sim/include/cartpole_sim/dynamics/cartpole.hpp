#ifndef CARTPOLE_HPP
#define CARTPOLE_HPP

#include <Eigen/Dense>

namespace cartpole_sim::dynamics {

class CartPole
{
  public:
    CartPole(double pendulum_mass, double cable_longitude, double gravity,
             double cart_mass);

    /**
        This function will compute the dynamics of the full system using the
        Lagrangian and resulting in the following formula: M(q)*q_dotdot + C(q,
        q_dot)*q_dot = t_g(q) + B*u

        Using the previous equation i can solve by solving for q_dotdot, solving
        for this vector leaves the formula as a linear system equation (Ax = b)
        being A the 2x2 matrix M(q), x being a 2x1 vector q_dotdot and b being
        the following operation: t_g(q) + B*u - C(q, q_dot)*q_dot
    */
    Eigen::Vector4d compute_dynamics(const Eigen::Vector4d& state) const;

    /**
        This function takes on the role of calculating the mechanical energy of
        the system it does not have an utility outside of corroborating the
        correctness of the simulation
    */
    double compute_mechanical_energy(const Eigen::Vector4d& state) const;
    double get_input_force();
    void set_input_force(double input_force);

  private:
    double pendulum_mass_;
    double cable_longitude_;
    double gravity_;
    double cart_mass_;
    double input_force_;
};

}  // namespace cartpole_sim::dynamics
#endif
