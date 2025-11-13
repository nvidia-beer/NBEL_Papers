# Neuromorphic Brain-Engineered Learning for Adaptive Robotic Control



## Introduction

This project investigates **neuromorphic adaptive control systems** using Spiking Neural Networks (SNNs) for robotic applications. Traditional control systems require accurate models and extensive calibration. This work demonstrates that brain-inspired neuromorphic computing can overcome these limitations through **online learning**, achieving real-time adaptation with extreme energy efficiency.

The research progresses from foundational neuromorphic implementations (2021) to fully adaptive predictive control (2024), demonstrating that SNNs can learn corrections in real-time—adapting to payloads, actuator failures, and model uncertainties without prior training on these scenarios.

**Key Results:**
- Online learning enables < 5% trajectory error in 1-2 seconds for payload adaptation
- Neuromorphic controllers match CPU performance for autonomous driving up to 15 m/s
- Adaptive MPC achieves 96% error reduction with continuous model correction
- 10-100× energy savings on neuromorphic hardware

---

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

- **[Prescribed Error Sensitivity (PES): Online Learning](papers/PES_online_learning.md)**  
   Mathematical foundations, stability analysis, and applications of the PES learning rule


---
