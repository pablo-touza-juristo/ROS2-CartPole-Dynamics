# Nonlinear Dynamics and Control of Underactuated Systems: The ROS 2 Cartpole

A high-fidelity C++20 framework for real-time physics simulation and LQR control of underactuated systems that
features Lagrangian dynamics, RK4 numerical integration, and Eigen3-accelerated linear algebra.

## Index

### 1. Introduction

### 2. Mathematical Modeling

### 3. Software Architecture

### 4. Verification and Validation

### 5. Environment and Reproducibility

## 1. Introduction

The "Inverted Pendulum on a Cart" is a fundamental benchmark in control theory
and underactuated robotics. As the system only possesses a single control input that
if the force applied to the cart, but two degrees of freedom (the cart's position
and the pendulum angle) it is unstable, non-linear and underactuated.

This project main focus is the development of high-fidelity physics simulation
implemented within the ROS 2 (Jazzy Jalisco) ecosystem. This implementation
emphasizes three core pillars:

- Numerical Stability: to ensure physical accuracy and energy conservation within
  the physics simulation loop I have implemented a 4th-order Runge-Kutta (RK4)
  integrator.

- Mathematical Rigor: to facilitate precise Linear Quadratic Regulator (LQR)
  controller design and easier physics implementation, I have derived the system's
  dynamics through the Lagrangian mechanics.

- Software Engineering: for implementing a modular, real-time capable architecture
  in C++20 I have adhered to the "ROS 2 C++ Style Guides", based on "Google's C++ Style
  Guides" and used the linear algebra library "Eigen3" for optimized matrix operations.

The final objective is to achieve a system that is capable of balancing in upright
position, as a demonstration of the seamless integration of the physical models with
modern robotics middleware and professional software standards.

## 2. Mathematical Modeling

To ensure the simulation's fidelity, the system is modeled using **Lagrangian mechanics**. This analytical approach allows for a precise derivation of the equations of motion (EoM), capturing the complex non-linear coupling between the cart and the pendulum.

### 2.1. System Parameters and Coordinates

The model consists of a cart of mass $M$ constrained to move along a horizontal track and a rigid pendulum of mass $m$ and length $l$ attached to the cart's pivot.

| Parameter       | Symbol |  Unit   | Description                                   |
| :-------------- | :----: | :-----: | :-------------------------------------------- |
| Cart Mass       |  $M$   |   kg    | Mass of the sliding base                      |
| Pendulum Mass   |  $m$   |   kg    | Mass of the rod and load                      |
| Pendulum Length |  $l$   |    m    | Distance from the pivot to the center of mass |
| Gravity         |  $g$   | $m/s^2$ | Gravitational acceleration (9.81)             |
| Input Force     |  $F$   |    N    | Horizontal force applied to the cart          |

### 2.2. State-Space Representation

The system state $\mathbf{x}$ is defined by grouping the generalized coordinates followed by their respective time derivatives. This structure is implemented using **Eigen3** vectors for optimized computation:

$$
\mathbf{x} = \begin{bmatrix} x \\ \theta \\ \dot{x} \\ \dot{\theta} \end{bmatrix}
$$

Where:

- **$x$**: Cart position on the horizontal axis.
- **$\theta$**: Pendulum angle (measured from the upright vertical position).
- **$\dot{x}$**: Linear velocity of the cart.
- **$\dot{\theta}$**: Angular velocity of the pendulum.

### 2.3. Equations of Motion (Manipulator Form)

The system dynamics are expressed in the standard robotic manipulator equation form, following the conventions of modern underactuated robotics literature:

$$
\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} = \boldsymbol{\tau}_g(\mathbf{q}) + \mathbf{B}u
$$

Given the generalized coordinates $\mathbf{q} = [x, \theta]^T$ and the scalar control input $u = F$, the matrices are defined as follows:

**Mass/Inertia Matrix $\mathbf{M}(\mathbf{q})$:**

$$
\mathbf{M}(\mathbf{q}) = \begin{bmatrix}
M + m & ml \cos\theta \\
ml \cos\theta & ml^2
\end{bmatrix}
$$

**Coriolis and Centrifugal Matrix $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$:**

$$
\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) = \begin{bmatrix}
0 & -ml\dot{\theta}\sin\theta \\
0 & 0
\end{bmatrix}
$$

**Gravity Vector $\boldsymbol{\tau}_g(\mathbf{q})$:**

$$
\boldsymbol{\tau}_g(\mathbf{q}) = \begin{bmatrix}
0 \\
-mgl \sin\theta
\end{bmatrix}
$$

**Input Actuation Matrix $\mathbf{B}$:**
The system is fundamentally underactuated, as $\text{rank}(\mathbf{B}) < \dim(\mathbf{q})$:

$$
\mathbf{B} = \begin{bmatrix}
1 \\
0
\end{bmatrix}
$$

### 2.4. Numerical Integration

These coupled non-linear equations are numerically integrated in real-time within the **ROS 2 Jazzy** node
using a **4th-order Runge-Kutta (RK4)** method. This ensures high numerical stability and energy
conservation during the simulation at a fixed frequency of **100Hz**.

### 2.5. Linear System Resolution

To isolate the joint accelerations ($\ddot{\mathbf{q}}$) required for the numerical integration, I treat
the equation of motion as a linear system $\mathbf{A}\mathbf{x} = \mathbf{b}$. Given that the mass matrix
$\mathbf{M}(\mathbf{q})$ is symmetric and positive-definite, I use **Eigen3's LDLT decomposition**
(`mass.ldlt().solve(...)`) to compute the accelerations efficiently. This method avoids the computational
cost and potential numerical instability of direct matrix inversions.

## 3. Software Architecture

To implement a robust and maintainable system, I have separated the pure mathematical physics from the ROS 2 middleware.

### 3.1. Core Modules (Pure C++)

- **`dynamics::CartPole`**: This class encapsulates the physical properties of the system and the Lagrangian dynamics solver.
- **`math::RK4Integrator`**: A generic Runge-Kutta 4th Order numerical integrator. For reusability, it accepts any
  standard C++ callable (`std::function`) that returns a state derivative.

### 3.2. ROS 2 Node and Memory Management

The `CartPoleNode` connects the physics engine with the ROS 2 environment, executing a strict 100Hz control loop ($\Delta t = 0.01s$).

To ensure real-time performance and avoid memory fragmentation during the execution of the node, I have implemented strict value
semantics. The `CartPole` and `RK4Integrator` objects are not allocated dynamically on the heap using pointers. Instead,
they are constructed and moved (`std::move`) directly into the node's contiguous memory space. This approach eliminates
pointer indirection overhead and ensures data locality.

## 4. Verification and Validation

To verify the physical accuracy and numerical stability of the RK4 integrator, I have included a mechanical
energy computation method in the `CartPole` class. By monitoring the sum of the kinetic and potential energy
of the system during the simulation, I can validate that the integrator does not artificially inject or leak
energy over time, excluding standard floating-point drift.

## 5. Environment and Reproducibility

### 5.1. Dependencies

- **ROS 2** (Jazzy Jalisco)
- **Eigen3** (Linear algebra)
- **C++20 Compiler**

### 5.2. Build Instructions

This project is built using the standard `colcon` build system:

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone <your-repository-url>
cd ..
colcon build --packages-select cartpole_sim
source install/setup.bash
```
