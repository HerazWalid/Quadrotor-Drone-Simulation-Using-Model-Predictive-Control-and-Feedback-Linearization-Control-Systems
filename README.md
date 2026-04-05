# 🚁 Modeling and Simulation of an Unmanned Aerial Vehicle (UAV)

## Simulation Video Link:
https://www.youtube.com/watch?v=N_0XCENQapY


<div align="center">

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a+-orange?style=for-the-badge&logo=mathworks)
![License](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen?style=for-the-badge)
![Institution](https://img.shields.io/badge/IGEE-Algeria-red?style=for-the-badge)

**Bachelor's Graduation Project — Institute of Electrical and Electronic Engineering (IGEE / ex-INELEC), University of Boumerdes, Algeria, 2024**

*by **HERAZ Walid** — supervised by **Dr. Tabet Youcef***

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Walid%20Heraz-0077B5?style=flat-square&logo=linkedin)](https://www.linkedin.com/in/walid-heraz-94337a240/)
[![GitHub](https://img.shields.io/badge/GitHub-HerazWalid-181717?style=flat-square&logo=github)](https://github.com/HerazWalid)

</div>

---

## 📖 Overview

This project presents a complete **modeling, simulation, and control system design** for a quadrotor Unmanned Aerial Vehicle (UAV) — implemented entirely in **MATLAB**. The drone is modeled as a nonlinear 6-degrees-of-freedom rigid body, and a **hierarchical dual-loop control architecture** is developed that combines:

- **Feedback Linearization** — for outer-loop position control (translational dynamics)
- **Model Predictive Control (MPC)** on an **LPV (Linear Parameter Varying)** model — for inner-loop attitude control (rotational dynamics)

The simulation validates the system across **four distinct 3D trajectories**, demonstrating robust tracking performance in all six degrees of freedom.

---

## 🎯 Objectives

- Derive a complete mathematical model of a quadrotor UAV from first principles (Newton–Euler formalism)
- Design a hierarchical control system combining nonlinear and predictive control techniques
- Simulate trajectory tracking for multiple 3D reference paths in MATLAB
- Visualize the drone's physical motion in an animated 3D environment

---

## 📐 System Architecture

The control system operates on **two nested loops** running at different sampling rates:

```
Trajectory Planner
      │
      ▼  (every 4×Ts)
┌─────────────────────────┐
│   Feedback Linearization │  ──►  U1  (Thrust)
│   (Outer Position Loop)  │  ──►  φ_ref, θ_ref, ψ_ref
└─────────────────────────┘
      │
      ▼  (every Ts)
┌─────────────────────────┐
│   MPC on LPV model       │  ──►  U2, U3, U4  (Roll, Pitch, Yaw torques)
│   (Inner Attitude Loop)  │
└─────────────────────────┘
      │
      ▼
┌─────────────────────────┐
│   Nonlinear Plant         │  ──►  12 States: [u,v,w, p,q,r, x,y,z, φ,θ,ψ]
│   (drone_plant.m / ODE45)│
└─────────────────────────┘
```

| Loop | Controller | Rate | Outputs |
|------|-----------|------|---------|
| Outer | Feedback Linearization | `4 × Ts` | `U1`, `φ_ref`, `θ_ref` |
| Inner | MPC (LPV model) | `Ts = 0.1 s` | `U2`, `U3`, `U4` |

The inner loop runs **4× faster** than the outer loop (`innerDyn_length = 4`) to give MPC sufficient time to converge the attitude to the references produced by the position controller.

---

## 🧮 Mathematical Modeling

### State Vector

The drone state is a 12-dimensional vector in the body frame:

```
states = [u, v, w, p, q, r, x, y, z, φ, θ, ψ]
```

| Symbol | Description |
|--------|-------------|
| `u, v, w` | Linear velocities in body frame |
| `p, q, r` | Angular rates in body frame |
| `x, y, z` | Inertial position |
| `φ, θ, ψ` | Roll, Pitch, Yaw (Euler angles) |

### Control Inputs

| Input | Description | Equation |
|-------|-------------|----------|
| `U1` | Total Thrust | `U1 = cT(Ω₁² + Ω₂² + Ω₃² + Ω₄²)` |
| `U2` | Roll Torque | `U2 = cT·l·(Ω₄² − Ω₂²)` |
| `U3` | Pitch Torque | `U3 = cT·l·(Ω₃² − Ω₁²)` |
| `U4` | Yaw Torque | `U4 = cQ·(−Ω₁² + Ω₂² − Ω₃² + Ω₄²)` |

### Equations of Motion (Newton–Euler)

**Translational dynamics (body frame):**
```
u̇ = vr − wq + g·sin(θ)
v̇ = wp − ur − g·cos(θ)·sin(φ)
ẇ = uq − vp − g·cos(θ)·cos(φ) + U1/m
```

**Rotational dynamics (with gyroscopic effects):**
```
ṗ = qr·(Iy−Iz)/Ix − (Jtp/Ix)·q·Ω + U2/Ix
q̇ = pr·(Iz−Ix)/Iy + (Jtp/Iy)·p·Ω + U3/Iy
ṙ = pq·(Ix−Iy)/Iz + U4/Iz
```

**Kinematics** are handled via the **Rotation matrix R(φ,θ,ψ)** (body → inertial) and the **Transfer matrix T(φ,θ)** (angular rates → Euler angle rates).

---

## 🏗️ Project Structure

```
📦 UAV-Simulation/
├── 📄 MAIN.m                    # Main simulation script (entry point)
├── 📄 drone_plant.m             # Nonlinear UAV plant model (ODE)
├── 📄 Feedback_Linearization.m  # Outer-loop position controller
├── 📄 LPV.m                     # Linear Parameter Varying model builder
├── 📄 MPC.m                     # MPC matrices generator (Hdb, Fdbt, Cdb, Adc)
├── 📄 trajectory_generator.m    # Reference trajectory generator (4 modes)
├── 📄 init_constants.m          # Central parameter/constant store
└── 📄 README.md
```

---

## 📂 File Descriptions

### `MAIN.m` — Entry Point
The master simulation script. Orchestrates the full control loop:
1. Loads constants and generates the reference trajectory
2. Runs the **outer loop** (Feedback Linearization) every `4×Ts`
3. Runs the **inner loop** (MPC on LPV) every `Ts`, 4 times per outer step
4. Integrates the nonlinear plant using `ode45`
5. Produces all plots and the 3D animation

### `drone_plant.m` — Nonlinear Plant
Defines the full 12-state nonlinear UAV ODE:
- Uses the rotation matrix `R` to map body velocities to inertial positions
- Uses the transfer matrix `T` to map body rates to Euler angle rates
- Includes gyroscopic effects via the global `omega_total`
- Called by `ode45` at each integration step

### `Feedback_Linearization.m` — Outer-Loop Controller
Implements the **feedback linearization** position controller:
- Takes `[X, Ẋ, Ẍ]`, `[Y, Ẏ, Ÿ]`, `[Z, Ż, Z̈]` references and the current state
- Computes position errors and applies a **PD-like stabilizing law** with gains derived from user-specified stable poles `px`, `py`, `pz`
- Outputs: `φ_ref`, `θ_ref`, and `U1` (thrust)

**Pole-to-gain mapping:**
```matlab
k1 = real((p1 - (p1+p2)/2)^2 - (p1+p2)^2/4)   % proportional gain
k2 = real(p1 + p2)                               % derivative gain
```

### `LPV.m` — Linear Parameter Varying Model
Reformulates the nonlinear rotational dynamics into an **LPV state-space** format:
- State: `[φ, φ̇, θ, θ̇, ψ, ψ̇]`
- Input: `[U2, U3, U4]`
- Output: `[φ, θ, ψ]`
- Nonlinearities are **encapsulated** in the A matrix (functions of current states), B stays constant
- Discretized using **Forward Euler**: `Ad = I + Ts·A`, `Bd = Ts·B`

### `MPC.m` — MPC Matrix Generator
Builds the condensed MPC cost function matrices for use with MATLAB's `quadprog`:

```
min  ½·ΔU'·Hdb·ΔU + f'·ΔU
 ΔU
```

- **Augmented state-space** formulation (`Ã`, `B̃`, `C̃`) with `ΔU` as the optimization variable
- Constructs block-diagonal weighting matrices `Qdb`, `Rdb` over the horizon `hz`
- Returns `Hdb` (Hessian), `Fdbt` (linear term), `Cdb` (prediction map), `Adc` (state propagation)

### `trajectory_generator.m` — Reference Path Generator
Generates smooth 3D reference trajectories with positions, velocities, and accelerations:

| ID | Name | Description |
|----|------|-------------|
| `1` | Straight Line | Linear ramp with sinusoidal altitude |
| `2` | **Circular/Spiral** (default) | Expanding helix — `x = (r/10·t+2)·cos(α)` |
| `3` | Moderate Circular | Gentle circle with altitude ramp |
| `4` | Crown Path | Sinusoidal up-down motion — most challenging |

Also computes the **yaw reference `ψ_ref`** aligned with the direction of motion, with proper wrapping to avoid discontinuities.

### `init_constants.m` — Central Parameter Store
Single source of truth for all simulation parameters. Returns a cell array:

```matlab
constants = {Ix, Iy, Iz, m, g, Jtp, Ts, Q, S, R, ct, cq, l,
             controlled_states, hz, innerDyn_length, px, py, pz, trajectory}
```

---

## ⚙️ Physical Parameters (AscTec Hummingbird UAV)

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Mass | `m` | 0.698 | kg |
| Moment of Inertia X | `Ix` | 0.0034 | kg·m² |
| Moment of Inertia Y | `Iy` | 0.0034 | kg·m² |
| Moment of Inertia Z | `Iz` | 0.006 | kg·m² |
| Propeller inertia | `Jtp` | 1.302×10⁻⁶ | N·m·s² |
| Thrust coefficient | `ct` | 7.6184×10⁻⁸ | N·s² |
| Drag coefficient | `cq` | 2.6839×10⁻⁹ | N·m·s² |
| Arm length | `l` | 0.171 | m |
| Gravity | `g` | 9.81 | m/s² |
| Sampling time | `Ts` | 0.1 | s |
| Inner loop ratio | — | 4 | — |
| MPC horizon | `hz` | 4 | steps |

---

## 🎛️ Controller Tuning

### Feedback Linearization Poles
Defined in `init_constants.m`. All three axes currently use stable real poles:
```matlab
px = [-1, -2];   % X-axis closed-loop poles
py = [-1, -2];   % Y-axis closed-loop poles
pz = [-1, -2];   % Z-axis closed-loop poles
```
Moving poles further left (more negative) increases tracking aggressiveness at the cost of larger transients.

### MPC Weight Matrices
```matlab
Q = diag([10, 10, 10]);   % Output error penalty (φ, θ, ψ)
S = diag([10, 10, 10]);   % Terminal state penalty
R = diag([10, 10, 10]);   % Control effort penalty (ΔU2, ΔU3, ΔU4)
```
- Increase `Q`/`S` relative to `R` for tighter angle tracking (larger control efforts)
- Increase `R` relative to `Q`/`S` for smoother inputs (slower response)

---

## 🚀 Getting Started

### Prerequisites
- **MATLAB** R2020a or later (recommended: R2023a+)
- **Optimization Toolbox** (required for `quadprog`)

### Running the Simulation

1. Clone or download this repository
2. Open MATLAB and navigate to the project folder
3. Open `init_constants.m` and set your desired trajectory:
   ```matlab
   trajectory = 2;  % 1=straight line, 2=spiral, 3=moderate circle, 4=crown
   ```
4. Run the main script:
   ```matlab
   >> MAIN
   ```

The simulation runs for **100 seconds** of simulated time. Expect runtime of several minutes depending on your hardware.

### Expected Outputs

After the simulation completes, MATLAB will display:
- **3D animated flight** — a physical drone model follows both the reference (blue dashed) and actual (red dotted) trajectory
- **Position tracking plots** — X, Y, Z vs. their references
- **Velocity tracking plots** — Ẋ, Ẏ, Ż vs. their references
- **Angle tracking plots** — φ, θ, ψ vs. their references
- **Control inputs plot** — U1 (N), U2, U3, U4 (N·m) over the first 10 seconds

---

## 📊 Key Results

All four trajectories were successfully tracked with **minimal steady-state error**:

| Trajectory | Tracking Quality | Notable Behavior |
|------------|-----------------|-----------------|
| Circular/Spiral | ✅ Excellent | Small initial overshoot, fast convergence |
| Straight Line | ✅ Excellent | Transient response well-damped |
| Crown Path | ⚠️ Good | Most challenging due to rapid Z oscillations |
| Spiral (expanding) | ✅ Excellent | Smooth tracking after alignment phase |

> **Note:** An initial transient overshoot occurs at the start of each simulation. This is expected — the drone begins at the origin `(0,0,0)` while the trajectory may start at a different position. Once the drone captures the reference path, tracking becomes smooth.

---

## 🔬 Control Theory Background

### Why LPV instead of linearization?

Classical Taylor-series linearization around a single operating point degrades as the system moves away from that point. The **LPV approach** instead *reformulates* the full nonlinear model so that its A matrix varies with the current state — no accuracy is lost, and stability need only be verified at the vertices of the operating region.

### Why MPC for attitude?

MPC solves a **finite-horizon optimization problem** at every time step, producing control increments `ΔU` that minimize a weighted sum of tracking errors and control effort. This makes it naturally suited for multi-variable systems like the drone's 3-axis attitude, while keeping inputs smooth. The `quadprog` solver handles the resulting QP efficiently.

### Why Feedback Linearization for position?

The position subsystem has a clean input–output structure that allows exact cancellation of nonlinearities, reducing the closed-loop dynamics to simple linear second-order systems. This makes pole placement straightforward and gives an analytical formula for the required thrust `U1` and reference angles.

---

## ⚠️ Known Limitations & Future Work

- **No disturbance rejection** — wind gusts or payload changes are not modeled
- **No sensor noise** — states are assumed perfectly measured
- **MPC weight matrices are equal** — proper LQR-style tuning could improve performance
- **Forward Euler discretization** — a ZOH or exact discretization would improve accuracy at larger `Ts`
- **No input/state constraints enforced** — the `quadprog` call has no bounds; real motors have saturation limits
- **Imaginary state detection** — a basic check is implemented but not handled gracefully

### Suggested Improvements
- Add a **Kalman filter** for state estimation under sensor noise
- Enforce **motor saturation constraints** in the MPC QP (`lb`/`ub` in `quadprog`)
- Implement **Backstepping** or **INDI** for robustness to disturbances
- Tune weight matrices using **LQR** or **multi-objective optimization**
- Deploy on a real drone: translate MATLAB code to **C/C++** via MATLAB Coder and flash to a flight controller

---

## 📚 References

1. Bresciani, T. — *Modelling, Identification and Control of a Quadrotor Helicopter*
2. Trapiello Fernández, C. — *Control of a UAV using LPV Techniques*, UPC-ETSEIB, 2018
3. Craig, J. J. — *Introduction to Robotics, Mechanics and Control*, 3rd ed., 2005
4. Meadows, E. S. & Rawlings, J. B. — *Model Predictive Control*
5. Henson, M. A. & Seborg, D. E. — *Feedback Linearization Control*
6. MathWorks — `quadprog` documentation: [mathworks.com](https://es.mathworks.com/help/optim/ug/quadprog.html)
7. aa4cc — *Introduction to MPC – reference tracking*, YouTube, 2017

---




The drone physical parameters are based on the **AscTec Hummingbird** quadrotor datasheet.  
The 3D MATLAB drone animation is adapted from the work of **Jitendra Singh** ([QuadCopter-Matlab-Animation-Code](https://github.com/jitendra825/QuadCopter-Matlab-Animation-Code)).

---

<div align="center">


</div>


## Author

**Walid Heraz**
Bachelor’s Thesis – Electrical and Electronic Engineering
University of Boumerdes
June 2024



