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
* `setup_parameters.m`: Defines the physical constants (masses, lengths, gravity) for the simulation.
* `nonlinear_crane_dynamics.m`: Contains the core ODE functions for the nonlinear system.
* `Part0_Controllability.m`: Script to verify if the system can be fully controlled.
* `Part1_LQR.m`: Implementation of the Linear Quadratic Regulator.
* `Part2_Observability.m`: Analysis of whether the states can be estimated from measurements.
* `Part3_Observer_Cases.m`: Implementation of Luenberger Observers and state estimation cases.
* `Part4_LQG.m`: Full state estimation and control using the Kalman Filter.
* `lqg_dynamics_wrapper.m`: Helper function for the LQG simulation.
* `Dual_Load_Crane_System_report.pdf`: Detailed documentation and analysis of the project.

---

## 💻 Getting Started

### Prerequisites
* **MATLAB**: Required to run the `.m` scripts.
* **Control System Toolbox**: Recommended for functions like `lqr()`, `ss()`, and `obsv()`.

### Installation & Execution
1. **Clone the repository**:
   ```bash
   git clone [https://github.com/pranavkolii/Optimal-Control-and-State-Estimation-of-a-Dual-Load-Crane-System.git](https://github.com/pranavkolii/Optimal-Control-and-State-Estimation-of-a-Dual-Load-Crane-System.git)
2. **Open MATLAB and navigate to the project directory.**
3. **Run the simulation**: Run the scripts in order (Part 0 through Part 4) to see the progression from system analysis to full LQG control.

---

## 📊 Results
The controllers and observers successfully demonstrate the following:
* **Precise Positioning**: The cart reaches the target coordinate with minimal overshoot.
* **Sway Suppression**: Effective damping of the double-pendulum oscillations during and after movement.
* **Robust Estimation**: Accurate state tracking is maintained even when only the cart position is measured and the system is subject to noise.

## Outcomes
* Developed and implemented an LQG control strategy for a nonlinear dual-load crane system, utilizing LQR for optimal state feedback and Luenberger observers.
* Conducted comparative simulations applying controllers to both linearized and original nonlinear models to verify local and global stability.
