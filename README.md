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

The system state $\mathbf{x}$ is defined by grouping the generalized coordinates followed by their respective time derivatives. This structure is implemented using **Eigen3** vectors for optimized memory alignment and computation:

$$
\mathbf{x} = \begin{bmatrix} x \\ \theta \\ \dot{x} \\ \dot{\theta} \end{bmatrix}
$$

Where:

- **$x$**: Cart position on the horizontal axis.
- **$\theta$**: Pendulum angle (measured from the upright vertical position).
- **$\dot{x}$**: Linear velocity of the cart.
- **$\dot{\theta}$**: Angular velocity of the pendulum.

### 2.3. Equations of Motion (Matrix Form)

The dynamics can be expressed in the standard robotic manipulator form, which is particularly useful for physics validation and future control implementation:

$$
\mathbf{M}(q)\ddot{q} + \mathbf{C}(q, \dot{q})\dot{q} + \mathbf{G}(q) = \boldsymbol{\tau}
$$

Given the generalized coordinates $q = [x, \theta]^T$, the matrices are defined as:

**Mass Matrix $\mathbf{M}(q)$:**

$$
\mathbf{M}(q) = \begin{bmatrix} M + m & ml \cos\theta \\ ml \cos\theta & ml^2 \end{bmatrix}
$$

**Coriolis and Centrifugal Vector $\mathbf{C}(q, \dot{q})\dot{q}$:**

$$
\mathbf{C}(q, \dot{q})\dot{q} = \begin{bmatrix} -ml\dot{\theta}^2 \sin\theta \\ 0 \end{bmatrix}
$$

**Gravity Vector $\mathbf{G}(q)$:**

$$
\mathbf{G}(q) = \begin{bmatrix} 0 \\ -mgl \sin\theta \end{bmatrix}
$$

**Generalized Forces $\boldsymbol{\tau}$:**

$$
\boldsymbol{\tau} = \begin{bmatrix} F \\ 0 \end{bmatrix}
$$

### 2.4. Numerical Integration

These coupled non-linear equations are numerically integrated in real-time within the **ROS 2 Jazzy** node using a **4th-order Runge-Kutta (RK4)** method. This ensures high numerical stability and energy conservation during the simulation at a fixed frequency of **100Hz**. The implementation follows the **Google C++ Style Guide** and leverages **Eigen3** for efficient matrix-vector operations.## 3. Software Architecture

## 4. Verification and Validation

## 5. Environment and Reproducibility
