# Neuromorphic Brain-Engineered Learning for Adaptive Robotic Control



## Introduction

This project investigates **neuromorphic adaptive control systems** using Spiking Neural Networks (SNNs) for robotic applications. Traditional control systems require accurate models and extensive calibration. This work demonstrates that brain-inspired neuromorphic computing can overcome these limitations through **online learning**, achieving real-time adaptation with extreme energy efficiency.

The research progresses from foundational neuromorphic implementations (2021) to fully adaptive predictive control (2024), demonstrating that SNNs can learn corrections in real-time—adapting to payloads, actuator failures, and model uncertainties without prior training on these scenarios.



## Research Papers

### 2021-2024 Research Sequence

1. **[2021: Neuromorphic Inverse Kinematics](papers/2021.md)**  
   ANN-to-SNN conversion, offline PES fine-tuning, Intel Loihi deployment for robotic inverse kinematics

2. **[2022: Adaptive Dynamics Compensation](papers/2022.md)**  
   **True online learning** using PES—real-time adaptation to unexpected payloads in wheelchair-mounted robotic arms

3. **[2023: Autonomous Driving Controllers](papers/2023.md)**  
   Neuromorphic implementations of Pure-pursuit, Stanley, PID, and MPC controllers for autonomous vehicle path-tracking  
   *Published in: Frontiers in Neurorobotics, August 2023*

4. **[2024: Continuous Adaptive Model Predictive Control](papers/2024.md)**  
   SNNs with MPC for **continuous model correction**—eliminating the need for perfect system models  
   *Published in: Neuromorphic Computing and Engineering (IOPScience), 2024*

### Supplementary Material

- **[Control Systems Fundamentals](papers/Control.md)**  
   Comprehensive introduction to PID, Pure-Pursuit, Stanley, and MPC controllers with mathematical formulations and tuning guidelines

- **[Kinematic Bicycle Model (KBM)](papers/KBM.md)**  
   Mathematical foundations, reference point comparisons, and implementation details of the vehicle dynamics model used in autonomous driving research

- **[Prescribed Error Sensitivity (PES): Online Learning](papers/PES.md)**  
   Mathematical foundations, stability analysis, and applications of the PES learning rule



\clearpage

## Neural Engineering Framework (NEF)

### Introduction

The Neural Engineering Framework (NEF) is a principled methodology for constructing computational networks that process, transform, and adapt real-valued vectors and signals. NEF is built on four foundational pillars:

- **Representation:** Encoding high-dimensional numerical variables as activity patterns in neuron-like units or ensemble populations.
- **Transformation:** Implementing mathematically defined functions and mappings by optimizing neural connection weights and decoders.
- **Dynamics:** Realizing time-dependent behavior and recurrent computations by embedding differential equations and control laws within the network structure.
- **Learning:** Online adjustment of decoding weights to minimize output error, supporting adaptive and robust system performance through explicit learning rules.

This architecture enables the translation of high-level computational objectives into rigorously specified, scalable network models.

### NEF Mathematical Notation Table

\footnotesize

| Notation | Description |
| :-- | :-- |
| $\mathbf{x}$ | Input/state vector (task-relevant variable) |
| $e_i$| Encoding vector (preferred direction or feature) of neuron$i$ |
| $\alpha_i$| Gain (sensitivity) for neuron$i$ |
| $J_i^{bias}$| Baseline current or bias for neuron$i$ |
| $a_i(\mathbf{x})$| Activity (filtered spike output or rate code) of neuron$i$ |
| $G_i[\cdot]$ | Static or dynamic activation nonlinearity (e.g., LIF) |
| $d_i$| Linear decoder for neuron$i$ |
| $\hat{\mathbf{x}}$ | Estimated or decoded variable |
| $h$ | Synaptic filtering kernel (often low-pass/exponential) |
| $w_{ij}$| Weight from neuron$i$(source, usually in ensemble A) to$j$ (target, in B) |
| $\otimes$ | Outer product operation ($\mathbf{a} \otimes \mathbf{b}$) |
| $f(\mathbf{x})$ | Function to approximate (target transformation) |
| $d^f_i$| Decoder for neuron$i$, optimized for $f(\mathbf{x})$ |
| $A, B$ | System matrices from control or dynamic equations |
| $\tau$ | Synaptic time constant |
| $I$ | Identity matrix |
| $V(t)$| Membrane/State variable for LIF unit at time$t$ |
| $R$ | Resistance or scaling constant in LIF model |
| $\Delta d_i$ | Decoder update (PES learning) |
| $\kappa$ | PES learning rate |
| $E$ | Error signal (target minus decoded output) |

\normalsize

### Representation (Encoding and Decoding)

A vector $\mathbf{x} \in \mathbb{R}^d$ is encoded as:

$$
a_i(\mathbf{x}) = G_i\left[ \alpha_i \langle e_i, \mathbf{x} \rangle + J_i^{\text{bias}} \right]
$$

Filter-convolved spike outputs (or rates) are linearly combined:

$$
\hat{\mathbf{x}} = \sum_{i=1}^{N} (a_i * h) d_i
$$

The decoders $d_i$are optimized to minimize the mean squared error between$\mathbf{x}$and$\hat{\mathbf{x}}$. Filtering ($h$) ensures signal smoothness and temporal continuity.

Noise in the reconstructed variable scales as $1/\sqrt{N}$with the number of neurons$N$.

### Transformation (Function Approximation and Connectivity)

Given a target function $f(\mathbf{x})$, decoders $d^f_i$ are fit (typically by least-squares) so that:

$$
\hat{f}(\mathbf{x}) = \sum_{i=1}^{N} (a_i * h) d^f_i
$$

Inter-ensemble synaptic weights are defined:

$$
w_{ij} = d^A_i \otimes e^B_j
$$

where the decoders and encoders of each ensemble map population activity across transforms. This architecture allows the neuromorphic realization of arbitrary continuous mappings.

### Dynamics (System Integration and Recurrent Computation)

To realize canonical state-space or control models:

$$
\frac{d\mathbf{x}}{dt} = A\mathbf{x}(t) + B\mathbf{u}(t)
$$

The NEF sets synaptic weights as:

$$
A' = \tau A + I, \qquad B' = \tau B
$$

enabling recurrent and input-driven state update, including pure integration or more general dynamic laws.

### Online Learning (Prescribed Error Sensitivity, PES)

Online adaptation minimizes instantaneous output errors:

$$
E = \text{target} - \hat{\mathbf{x}}
$$

with each decoder updated locally:

$$
\Delta d_i = \kappa E a_i
$$

PES convergence rate depends on the learning rate ($\kappa$) and neural activity; error vanishes exponentially provided suitable conditions.

