# 🛩️ Modeling and Simulation of a Quadrotor UAV

# Simulation Video Link:
https://www.youtube.com/watch?v=N_0XCENQapY

## Overview

This project focuses on the **mathematical modeling**, **dynamic simulation**, and **control** of a quadrotor Unmanned Aerial Vehicle (UAV). A complete **6-degree-of-freedom (6-DOF)** nonlinear dynamic model is derived using Newton–Euler formalism, incorporating translational and rotational dynamics, gyroscopic effects, and actuator forces.

To ensure accurate trajectory tracking and robust stabilization, a hybrid control architecture combining **Model Predictive Control (MPC)** and **Feedback Linearization** is implemented and validated through MATLAB simulations.

---

## Key Features

* Full nonlinear **6-DOF quadrotor model**
* Rigid body dynamics
* Rotation matrix and transfer matrix formulation
* Force and torque modeling
* Gyroscopic effect modeling
* State-space representation
* **Model Predictive Control (MPC)**
* **Feedback Linearization Control**
* 4th Order Runge-Kutta (RK4) numerical integration
* MATLAB-based simulation environment
* Trajectory tracking and stabilization

---

## System Modeling

The quadrotor is modeled as a rigid body with six degrees of freedom:

### States

* Position: ( X, Y, Z )
* Orientation (Euler angles): ( \phi, \theta, \psi )
* Linear velocities: ( u, v, w )
* Angular velocities: ( p, q, r )

### Inputs

* ( U_1 ): Total thrust
* ( U_2 ): Roll torque
* ( U_3 ): Pitch torque
* ( U_4 ): Yaw torque

The model includes:

* Translational dynamics
* Rotational dynamics
* Coriolis and centripetal effects
* Gravitational forces
* Gyroscopic effects
* Actuator dynamics

---

## Control Strategy

A hybrid control architecture is used:

### 1. Model Predictive Control (MPC)

* Handles multi-variable control
* Anticipates future states
* Optimizes control inputs
* Enforces constraints
* Ensures smooth trajectory tracking

### 2. Feedback Linearization

* Cancels nonlinearities
* Converts nonlinear system into linear subsystems
* Provides fast and stable position control

This combination allows:
✔ High precision
✔ Stability
✔ Robustness to disturbances

---

## Simulation Setup

* Software: **MATLAB**
* Integration method: **4th Order Runge-Kutta (RK4)**
* Time step: 0.1 s
* Initial conditions defined for position, velocity, and orientation
* Multiple reference trajectories tested

---

## Results

The simulation results demonstrate:

* Accurate trajectory tracking
* Stable attitude control
* Smooth control inputs
* Robustness against nonlinear effects
* Improved flight precision

Plots include:

* Position tracking
* Velocity evolution
* Euler angles
* Control inputs

--- 

## Requirements

* MATLAB R2021+ (recommended)
* Control System Toolbox
* Optimization Toolbox
* Model Predictive Control Toolbox

---


## Future Work

* Real-time implementation on embedded hardware
* State estimation using Kalman Filters
* Obstacle avoidance
* Wind disturbance modeling
* Energy-aware control
* ROS/Gazebo integration

---

## Author

**Walid Heraz**
Bachelor’s Thesis – Electrical and Electronic Engineering
University of Boumerdes
June 2024



