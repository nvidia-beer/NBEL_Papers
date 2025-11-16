# Introduction to Control Systems

## Fundamental Control Algorithms for Path Tracking and Process Control

**This document provides an introductory overview of fundamental control algorithms used in robotics, automation, and industrial process control.**

## Notation Table

\footnotesize

| Notation | Description |
| :-- | :-- |
| $u(t)$ | Control signal (output) |
| $e(t)$ | Error signal (difference between desired and actual states) |
| $r(t)$ | Reference signal (desired setpoint or target) |
| $y(t)$ | System output (measured state or process variable) |
| $K_p$ | Proportional gain |
| $K_i$ | Integral gain |
| $K_d$ | Derivative gain |
| $t$ | Time variable |
| $\tau$ | Integration variable |
| $N$ | Prediction horizon (number of future time steps) |
| $x_k$ | State vector at discrete time step $k$ |
| $u_k$ | Control input at discrete time step $k$ |
| $J$ | Cost function (objective to minimize) |
| $Q$ | State cost weighting matrix |
| $R$ | Control effort weighting matrix |
| $S$ | Control rate weighting matrix |
| $f(\cdot)$ | System dynamics model |

\normalsize

## 1. Proportional-Integral-Derivative (PID) Controller

### **Introduction**

The **Proportional-Integral-Derivative (PID) controller** is the most widely used feedback control algorithm in industrial automation and process control. It continuously calculates an error value as the difference between a desired setpoint and a measured process variable, then applies a correction based on proportional, integral, and derivative terms.

PID control is fundamental to maintaining stable and accurate performance across diverse applications including temperature regulation, pressure control, flow management, motor speed control, and chemical process regulation.

### **Control Principle**

The PID controller operates on three complementary control actions:

- **Proportional (P):** Produces a control output proportional to the current error. Higher proportional gain increases system responsiveness but may cause oscillations.

- **Integral (I):** Accumulates past errors over time to eliminate steady-state offset. This term ensures the system eventually reaches the setpoint, though excessive integral action can cause overshoot and instability.

- **Derivative (D):** Predicts future error trends by computing the rate of change of error. This dampens oscillations and improves stability, particularly during rapid system changes, though it can amplify measurement noise.

### **Mathematical Formulation**

The continuous-time PID control law is:

$$
u(t) = K_p e(t) + K_i \int_0^{t} e(\tau) \, d\tau + K_d \frac{de(t)}{dt}
$$

where the error signal is:

$$
e(t) = r(t) - y(t)
$$

### **Discrete-Time Implementation**

For digital control systems with sampling period $\Delta t$:

$$
u_k = K_p e_k + K_i \sum_{j=0}^{k} e_j \Delta t + K_d \frac{e_k - e_{k-1}}{\Delta t}
$$

### **Tuning Considerations**

Proper tuning of the three gains ($K_p$, $K_i$, $K_d$) is critical for optimal performance. Common tuning methods include:

- **Manual tuning:** Iterative adjustment until satisfactory performance
- **Ziegler-Nichols method:** Systematic empirical approach
- **Auto-tuning algorithms:** Automated parameter identification

### **Applications**

- **Temperature control:** HVAC systems, ovens, reactors
- **Pressure regulation:** Pneumatic and hydraulic systems
- **Flow control:** Pumps, valves, chemical dosing
- **Position control:** Robotics, CNC machines, actuators
- **Speed control:** Motors, conveyor systems

## 2. Pure-Pursuit Path Following

### **Introduction**

**Pure-Pursuit** is a geometric path-tracking algorithm widely used in mobile robotics and autonomous navigation. It calculates control commands to steer an agent toward a goal point located at a fixed lookahead distance along a reference trajectory.

The algorithm is named for its behavior of continuously "pursuing" a moving target point that stays ahead of the robot on the desired path, similar to a donkey following a carrot held at a constant distance.

### **Control Principle**

The Pure-Pursuit controller operates by:

1. Identifying the robot's current position and heading
2. Finding the closest point on the reference path
3. Selecting a **lookahead point** at distance $l_d$ ahead along the path
4. Calculating the curvature of an arc connecting the robot to the lookahead point
5. Computing the control command to follow this arc

As the robot moves, a new lookahead point is continuously selected, causing the robot to trace the reference path.

### **Mathematical Formulation**

Given a reference path and current agent position, the lookahead point is identified at distance $l_d$ along the path. The angle $\alpha$ between the agent's heading and the lookahead direction determines the control output.

For a system with characteristic length $L$:

$$
u(t) = \arctan\left(\frac{2L \sin \alpha}{l_d}\right)
$$

where:
- $\alpha$: Angle to lookahead target (radians)
- $l_d$: Lookahead distance
- $L$: System characteristic length (e.g., wheelbase)

### **Tuning Considerations**

The lookahead distance $l_d$ is the primary tuning parameter:

- **Small $l_d$:** Aggressive tracking, fast response to path changes, potential for oscillations and instability
- **Large $l_d$:** Smooth tracking, slower response, tendency to cut corners on sharp turns
- **Adaptive $l_d$:** Dynamically adjust based on velocity, e.g., $l_d = k_v \cdot v(t)$

### **Applications**

- **Mobile robot navigation:** Warehouse robots, delivery robots
- **Autonomous vehicles:** Lane keeping, trajectory following
- **Agricultural automation:** Harvesting machines, field navigation
- **Aerospace:** UAV waypoint navigation
- **Marine vessels:** Ship autopilot systems

## 3. Stanley Lateral Controller

### **Introduction**

The **Stanley controller** is a path-tracking algorithm developed by Stanford University's team for the 2005 DARPA Grand Challenge, where it achieved the fastest completion time. Unlike Pure-Pursuit, which tracks a lookahead point, Stanley minimizes both **cross-track error** (lateral displacement from the reference path) and **heading error** (angular deviation from the desired orientation).

### **Control Principle**

The Stanley controller combines two error correction mechanisms:

1. **Heading alignment:** Corrects angular deviation to align the robot with the path direction
2. **Lateral correction:** Reduces perpendicular distance from the path, scaled by velocity to prevent overcorrection at high speeds

The controller typically uses the front axle as the reference point for error calculation.

### **Mathematical Formulation**

$$
u(t) = \psi(t) + \arctan\left(\frac{k \cdot e(t)}{k_s + v(t)}\right)
$$

where:
- $e(t)$: Cross-track error (lateral displacement from path)
- $\psi(t)$: Heading error (angular deviation from path tangent)
- $v(t)$: System velocity
- $k$: Cross-track error gain
- $k_s$: Softening constant (prevents singularity at zero velocity)

### **Physical Interpretation**

- **First term $\psi(t)$:** Directly corrects heading to align with reference direction
- **Second term:** Applies lateral correction proportional to displacement, automatically reducing at higher velocities to maintain stability

### **Tuning Considerations**

- $k \in [0.5, 5.0]$: Higher values increase lateral correction aggressiveness
- $k_s \in [0.1, 1.0]$: Prevents division by zero at low speeds
- Balance between tracking accuracy and control smoothness

### **Applications**

- **Autonomous driving:** Lane keeping, highway navigation
- **Robotics:** Precise trajectory following in structured environments
- **Racing applications:** High-speed path tracking
- **Agricultural vehicles:** Row following in crop fields

## 4. Model Predictive Control (MPC)

### **Introduction**

**Model Predictive Control (MPC)**, also known as **Receding Horizon Control**, is an advanced optimal control technique that uses a mathematical model of the system to predict future behavior over a finite time horizon. At each time step, MPC solves an optimization problem to find the control sequence that minimizes a cost function while respecting system constraints.

Unlike reactive control methods (such as PID), MPC is **proactive**—it anticipates future disturbances, setpoint changes, and constraint violations, enabling more sophisticated control strategies.

### **Control Principle**

The MPC methodology follows a repeating cycle:

1. **Measure:** Obtain current system state
2. **Predict:** Use system model to forecast future states over horizon $N$
3. **Optimize:** Solve constrained optimization to find control sequence minimizing cost
4. **Apply:** Implement only the first control action
5. **Shift horizon:** Move time window forward and repeat at next time step

This "receding horizon" strategy provides feedback and allows the controller to adapt to disturbances and changing conditions.

### **Mathematical Formulation**

At time $t$, solve the optimization problem:

$$
\min_{\{u_k\}_{k=0}^{N-1}} J = \sum_{k=0}^{N-1} \left( \| x_k - x_{\text{ref}} \|_Q^2 + \| u_k \|_R^2 + \| u_k - u_{k-1} \|_S^2 \right)
$$

subject to:

$$
x_{k+1} = f(x_k, u_k), \quad k = 0, 1, \ldots, N-1
$$

$$
x_{\min} \leq x_k \leq x_{\max}, \quad u_{\min} \leq u_k \leq u_{\max}
$$

### **Cost Function Components**

- $\| x_k - x_{\text{ref}} \|_Q^2$: **State tracking error** — penalizes deviation from reference trajectory, weighted by matrix $Q$

- $\| u_k \|_R^2$: **Control effort penalty** — discourages excessive actuator use, weighted by matrix $R$

- $\| u_k - u_{k-1} \|_S^2$: **Control smoothness penalty** — promotes gradual control changes, reducing wear and oscillations, weighted by matrix $S$

### **Key Features**

**Constraint Handling:** MPC explicitly incorporates hard constraints on states and controls (e.g., actuator limits, safety boundaries, physical limitations), which are difficult to handle in traditional PID control.

**Multivariable Control:** Naturally handles multiple inputs and outputs (MIMO systems) with coupling between variables.

**Predictive Capability:** Uses future reference trajectory and disturbance forecasts when available.

**Flexibility:** Cost function can be customized for specific objectives (tracking, regulation, economic optimization).

### **Optimization Methods**

- **Quadratic Programming (QP):** For linear systems with quadratic cost
- **Nonlinear Programming:** For nonlinear dynamics
- **Gradient-based methods:** Iterative descent algorithms
- **Convex optimization:** Fast solvers for convex problems

### **Tuning Considerations**

- **Prediction horizon $N$:** Longer horizons improve disturbance rejection but increase computational cost
- **Weighting matrices $Q, R, S$:** Balance tracking performance, control effort, and smoothness
- **Constraint specification:** Define operational limits and safety boundaries
- **Model accuracy:** Predictions depend on model fidelity

### **Applications**

- **Chemical process control:** Distillation columns, reactors, refineries
- **Automotive:** Adaptive cruise control, stability control, energy management
- **Aerospace:** Flight control, trajectory optimization, satellite attitude control
- **Robotics:** Manipulation planning, legged locomotion
- **Energy systems:** Power grid management, building HVAC, battery charging
- **Manufacturing:** Batch processes, quality control

## Comparison of Control Methods

\footnotesize

| **Feature** | **PID** | **Pure-Pursuit** | **Stanley** | **MPC** |
|:--|:--|:--|:--|:--|
| **Complexity** | Low | Low-Medium | Medium | High |
| **Constraint handling** | Limited | None | None | Explicit |
| **Predictive** | No | No | No | Yes |
| **Tuning difficulty** | Medium | Low | Medium | High |
| **Computational cost** | Very low | Low | Low | High |
| **Multivariable** | Limited | Single-input | Single-input | Native |
| **Typical applications** | Process control | Path following | Path tracking | Advanced control |

\normalsize
