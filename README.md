# Optimal Control and State Estimation of a Dual-Load Crane System

This project focuses on the modeling, control, and state estimation of an underactuated gantry crane system carrying two independent loads. The primary objective is to move the cart to a target position while suppressing the oscillations (sway) of both payloads using advanced control strategies and state estimation techniques.

---

## 🛠 System Description
The system consists of a frictionless cart (mass $M$) moving along a 1D track, actuated by an external force $F$. Two loads (masses $m_1$ and $m_2$) are suspended from the cart via cables of lengths $L_1$ and $L_2$.

### State Variables
The system is defined by a 6-dimensional state vector:
* **$x$**: Cart position
* **$\dot{x}$**: Cart velocity
* **$\theta_1, \theta_2$**: Sway angles of mass 1 and mass 2
* **$\dot{\theta}_1, \dot{\theta}_2$**: Angular velocities of mass 1 and mass 2

---

## 🚀 Features
* **Linearization**: Approximates nonlinear dynamics around the downward equilibrium for control design.
* **LQR Control**: Implements a Linear Quadratic Regulator to balance state error and control effort.
* **State Estimation**: 
    * **Luenberger Observer**: For deterministic state estimation.
    * **Kalman Filter**: For optimal estimation in the presence of stochastic noise.
* **LQG Control**: Integration of LQR and Kalman Filtering for robust performance under uncertainty.

---

## 📂 Project Structure
* `open_loop_system.m`: Simulates the system without feedback.
* `lqr_control.m`: Implementation of the LQR controller for point-to-point movement.
* `kalman_filter.m`: Script for state estimation in noisy environments.
* `lqg_control.m`: Integration of optimal control and estimation.

---

## 💻 Getting Started
1. **Clone the repository**:
   ```bash
   git clone [https://github.com/pranavkolii/Optimal-Control-and-State-Estimation-of-a-Dual-Load-Crane-System.git](https://github.com/pranavkolii/Optimal-Control-and-State-Estimation-of-a-Dual-Load-Crane-System.git)
