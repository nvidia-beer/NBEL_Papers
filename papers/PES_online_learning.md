# Prescribed Error Sensitivity (PES): Online Learning in Neuromorphic Control

## Overview

This document explains **Prescribed Error Sensitivity (PES)**, the primary online learning method used across the 2021-2024 neuromorphic control research sequence for adaptive control applications.

### PES Applications in the Research Sequence

1. **2021:** Offline fine-tuning of ANN-to-SNN converted networks for inverse kinematics
2. **2022:** **Online** real-time adaptation to payloads and disturbances in robotic arm control
3. **2024:** **Online** model-correcting adaptive MPC for autonomous vehicle control

**Evolution of PES Usage:**
- **2021:** PES used for offline calibration after ANN-to-SNN conversion
- **2022-2024:** PES enables true **online learning** - real-time adaptation during operation without pre-training on the specific disturbances/errors encountered

---

## 1. Mathematical Foundation of PES

### Problem Addressed

**Adaptive compensation** for model errors, disturbances, and unmodeled dynamics through **real-time weight updates** driven by observed errors.

### Core Learning Rule

**PES Learning Rule:**

$$
\Delta \mathbf{w} = -\kappa \mathbf{e} \mathbf{a}
$$

where:
- $\mathbf{w} \in \mathbb{R}^n$ are decoder weights from pre-synaptic neurons to output
- $\kappa$ is the learning rate (typically $10^{-6}$ to $10^{-4}$)
- $\mathbf{e}$ is the error signal (desired - actual)
- $\mathbf{a} \in \mathbb{R}^n$ is the pre-synaptic neural activity vector

**Interpretation:** This is a **correlation-based learning rule**—weights increase for neurons that fire when errors occur, creating associations between neural states and corrective actions.

### Matrix Form (Multi-Dimensional Outputs)

For outputs $\mathbf{y} \in \mathbb{R}^m$:

$$
\Delta \mathbf{W} = -\kappa \mathbf{e} \mathbf{a}^T
$$

where $\mathbf{W} \in \mathbb{R}^{n \times m}$ and the update is an outer product.

### Continuous-Time Dynamics

$$
\frac{d\mathbf{w}}{dt} = -\kappa \mathbf{e}(t) \mathbf{a}(t)
$$

with temporal filtering through synaptic time constants $\tau$:

$$
\tau \frac{d\mathbf{e}_{\text{filtered}}}{dt} = -\mathbf{e}_{\text{filtered}} + \mathbf{e}(t)
$$

---

## 2. Neural Engineering Framework (NEF) Integration

PES operates within the NEF paradigm, which provides a unified framework for neural computation:

### 1. Encoding: Continuous Variables → Neural Activities

$$
a_i(x) = G_i[\alpha_i \cdot x + J_{\text{bias},i}]
$$

where:
- $a_i(x)$ is the firing rate of neuron $i$
- $G_i[\cdot]$ is the neuron's activation function (typically LIF)
- $\alpha_i$ is the neuron's preferred direction vector (encoder)
- $J_{\text{bias},i}$ is the background current

### 2. Decoding: Neural Activities → Reconstructed Output

$$
\hat{y} = \sum_i w_i a_i(x) = \mathbf{w}^T \mathbf{a}
$$

The decoder weights $\mathbf{w}$ map neural activities to output estimates.

### 3. Learning: Error Signal Updates Decoders

$$
\mathbf{e} = y_{\text{desired}} - \hat{y}
$$

$$
\Delta \mathbf{w} = -\kappa \mathbf{e} \mathbf{a}
$$

PES adjusts the decoder weights $\mathbf{w}$ to minimize this error over time.

---

## 3. Stability Analysis

### Convergence Condition

For stable learning, the eigenvalues of the system's Jacobian must satisfy:

$$
\gamma = a - \kappa \|\delta\|^2 > -1
$$

where:
- $a$ is ensemble activity magnitude
- $\delta$ is filtered spike rate
- $\gamma$ determines stability

This ensures weight updates don't cause oscillations or divergence.

### Practical Stability Measures

1. **Learning Rate Constraints:** $\kappa \ll 1$ ensures gradual adaptation
2. **Synaptic Filtering:** Large $\tau$ (50-200ms) smooths weight changes
3. **Activity Regularization:** Prevents neurons from saturating

### Theoretical Foundation

PES is a **supervised Hebbian rule** with error modulation:

$$
\Delta w_{ij} = -\kappa \cdot \underbrace{e_j}_{\text{post-synaptic error}} \cdot \underbrace{a_i}_{\text{pre-synaptic activity}}
$$

This is equivalent to **gradient descent on output error**:

$$
\mathcal{L} = \frac{1}{2} \|\mathbf{e}\|^2 = \frac{1}{2} \|\mathbf{y}_{\text{target}} - \mathbf{W}^T \mathbf{a}\|^2
$$

$$
\frac{\partial \mathcal{L}}{\partial w_{ij}} = -e_j a_i
$$

Thus: $\Delta \mathbf{W} = -\kappa \nabla_\mathbf{W} \mathcal{L}$

### Convergence Conditions

For linear systems $\hat{\mathbf{y}} = \mathbf{W}^T \mathbf{a}$, PES converges if:

$$
\kappa < \frac{2}{\lambda_{\max}(\mathbf{A}^T \mathbf{A})}
$$

where $\mathbf{A}$ is the activity correlation matrix $\mathbb{E}[\mathbf{a}\mathbf{a}^T]$.

---

## 4. PES Applications Across 2021-2024

### 4.1 PES for Inverse Kinematics (2021) - **OFFLINE Fine-Tuning**

**Context:** Fine-tuning spiking neural networks after ANN-to-SNN conversion to recover accuracy lost during conversion.

**Training Pipeline:**
1. **Offline ANN Training:** Train fully-connected/ResNet networks on 200,000 IK samples
2. **ANN-to-SNN Conversion:** Convert trained ANN to SNN using NengoDL
3. **Offline PES Fine-Tuning:** Use PES to calibrate SNN weights using the training dataset
4. **Deployment:** Deploy fine-tuned SNN to Intel Loihi for inference

**Architecture:**

```
Input: End-effector position x ∈ ℝ³
       ↓
Ensemble: LIF neurons with CosineSimilarity encoding
       ↓
Decoder: w ∈ ℝ^{n×6} (n neurons → 6 joint angles)
       ↓
Output: Joint angles q ∈ ℝ⁶
       ↓
Forward Kinematics: x̂ = FK(q)
       ↓
Error: e = x_desired - x̂
       ↓
PES Update (offline): Δw = -κ e a
```

**Learning Signal:** Forward kinematics error (task space)

$$
\mathbf{e}_{\text{FK}} = \mathbf{x}_{\text{desired}} - \text{FK}(\mathbf{q}_{\text{output}})
$$

**Purpose:** Offline calibration to match ANN performance after neuromorphic conversion.

**Learning Type:** **Offline** - PES fine-tuning performed on training data before deployment, not during operation.

**Parameters:**
- Neurons: 128-256 per layer
- Learning rate: $\kappa \approx 10^{-6}$
- Hardware: Intel Loihi (for deployment)
- Training data: Same 200,000 samples used for ANN training

**Results:**
- Sub-millimeter accuracy (matching ANN performance)
- Real-time inference on neuromorphic hardware
- Energy-efficient compared to traditional computing

**Key Limitation:** Does not adapt to new situations during operation - weights are fixed after offline fine-tuning.

---

### 4.2 PES for Adaptive Dynamics Compensation (2022) - **ONLINE Learning**

**Context:** Real-time adaptation to unexpected payloads and configuration changes in wheelchair-mounted robotic arm.

**Key Innovation:** This is the first work to use PES for **true online learning** - adapting during operation to disturbances that were **not in the training data**.

**Architecture:**

```
Input: State [q, q̇] ∈ ℝ^{12} (6 angles + 6 velocities)
       ↓
Adaptive Ensemble: 1000 LIF neurons per ensemble
       ↓
Learning Connection (PES): W ∈ ℝ^{1000×6}
       ↓
Output: Corrective torques τ_adaptive ∈ ℝ⁶
       ↓
Combined Control: τ_total = τ_OSC + τ_adaptive
       ↓
Apply to Robot → Observe trajectory error
       ↓
Error Signal: e = (x_desired - x_actual, v_desired - v_actual)
       ↓
PES Update: Δw = κ e a
```

**Learning Signal:** Trajectory tracking error (task space position + velocity)

$$
\mathbf{e}_{\text{tracking}} = \begin{bmatrix} \mathbf{x}_{\text{desired}} - \mathbf{x}_{\text{actual}} \\ \dot{\mathbf{x}}_{\text{desired}} - \dot{\mathbf{x}}_{\text{actual}} \end{bmatrix}
$$

**Key Innovation:** Learns **corrective forces** to compensate for:
- Unexpected payload mass (0-1 kg)
- Changed inertia and dynamics
- Model inaccuracies

**Parameters:**
- Neurons per ensemble: 1000
- Learning rate: $\kappa = 10^{-6}$
- Synaptic time constants: $\tau_{\text{input}} = 12\text{ms}$, $\tau_{\text{output}} = 200\text{ms}$
- Adaptation time: 1-2 seconds to full compensation
- **No pre-training on payload scenarios**

**Performance (Online Learning):**
- Payload compensation: < 5% trajectory deviation with 1 kg load
- Convergence: 1-2 seconds **after payload is applied**
- Energy: Orders of magnitude less than traditional computing (Intel Loihi)
- **Adapts to completely new disturbances not in training data**

---

### 4.3 PES for Model-Correcting MPC (2024) - **ONLINE Model Correction**

**Context:** Real-time correction of bicycle model errors for autonomous vehicle control.

**Key Innovation:** PES learns to correct model dynamics **during driving** - no pre-training on actual vehicle behavior, only nominal bicycle model.

**Architecture:**

```
Input: [s_t, u_t] ∈ ℝ^{11} (9D state + 2D control)
       ↓
Adaptive Ensemble: 5-5000 LIF neurons
       ↓
Learning Connection (PES): W ∈ ℝ^{n×8}
       ↓
SNN Output: f_SNN(s,u) ∈ ℝ⁸ (dynamics correction)
       ↓
Hybrid Prediction: ṡ_pred = f_bicycle(s,u) + f_SNN(s,u)
       ↓
Integration: s_{t+1} = s_t + ṡ_pred Δt
       ↓
Compare with actual: ṡ_real (from CARLA)
       ↓
Dynamic Error: e_dynamic = ṡ_pred - ṡ_real
       ↓
PES Update: ΔW = -κ e_dynamic a^T
```

**Learning Signal:** **Dynamic error** (error in state derivatives, not states)

$$
\mathbf{e}_{\text{dynamic}} = \dot{\mathbf{s}}_{\text{predicted}} - \dot{\mathbf{s}}_{\text{real}}
$$

where $\dot{\mathbf{s}} = [\dot{x}, \dot{y}, \dot{\theta}, \dot{v}^x, \dot{v}^y, \dot{a}^x, \dot{a}^y, \dot{v}^r]^T \in \mathbb{R}^8$

**Key Innovation:** Learns to correct **model dynamics** (not just position errors), enabling accurate multi-step MPC predictions.

**Why Dynamic Error Matters:**

Traditional approaches minimize position error:
$$
\mathbf{e}_{\text{position}} = \mathbf{x}_{\text{predicted}} - \mathbf{x}_{\text{actual}}
$$

But for MPC, we need accurate **dynamics prediction** over horizon $N$:
$$
\mathbf{x}(t+k+1) = \mathbf{x}(t+k) + \dot{\mathbf{x}}(t+k) \Delta t
$$

If $\dot{\mathbf{x}}_{\text{predicted}}$ is wrong, errors compound exponentially:
$$
\|\mathbf{e}(t+k)\| \approx \|\mathbf{e}(t)\| \left\|\frac{\partial f}{\partial x}\right\|^k
$$

By learning to correct $\dot{\mathbf{s}}$ directly, the SNN ensures accurate predictions at all future steps $k = 0, ..., N-1$.

**Parameters:**
- Neurons: 5, 100, 1000, 5000 (scalability study)
- Learning rate: $\kappa = 10^{-4}$ (higher than 2022 due to faster vehicle dynamics)
- Synaptic time constant: $\tau = 50\text{ms}$
- Prediction horizon: $N = 5$ steps (0.5s lookahead)
- Timestep: $\Delta t = 0.1\text{s}$
- **No pre-training on actual vehicle dynamics**

**Performance (Online Learning):**
- **5 neurons:** 89.15% error reduction
- **100 neurons:** 93.5% reduction
- **1000 neurons:** 95.2% reduction
- **5000 neurons:** 96.08% reduction
- Convergence: 10-15 seconds **during initial driving** to 95% correction
- Fault tolerance: Handles 25% steering bias seamlessly
- **Learns corrections continuously while driving**

---

## 5. Performance Summary Across Papers

### 2021: Inverse Kinematics - **OFFLINE Fine-Tuning**

| Learning Type | Neurons | Accuracy | Training | Hardware |
|---------------|---------|----------|----------|----------|
| **Offline PES Fine-Tuning** | 128-256/layer | Sub-millimeter | Offline (ANN) + Offline PES Calibration | Intel Loihi |

**Result:** PES enables neuromorphic deployment with maintained accuracy after ANN-to-SNN conversion.

**Limitation:** Fixed weights after deployment - no adaptation to new situations during operation.

---

### 2022: Adaptive Dynamics Compensation - **ONLINE Learning**

| Learning Type | Neurons | Payload Compensation | Adaptation Time | Hardware |
|---------------|---------|---------------------|-----------------|----------|
| **Online PES** | 1000/ensemble | < 5% error with 1kg | 1-2 seconds | Intel Loihi |

**Result:** First true online PES learning - adapts in real-time to unexpected disturbances (payloads, configuration changes) **not in training data**.

**Breakthrough:** Demonstrates PES can learn during operation without pre-training on specific scenarios.

---

### 2024: Model-Correcting MPC - **ONLINE Model Correction**

| Learning Type | Neurons | Error Reduction | Convergence Time | Fault Tolerance | Platform |
|---------------|---------|-----------------|------------------|-----------------|----------|
| **Online PES** | 5 | 89.15% | 10-15 seconds | 25% steering bias | CARLA Simulator |
| **Online PES** | 100 | 93.5% | 10-15 seconds | 25% steering bias | CARLA Simulator |
| **Online PES** | 1000 | 95.2% | 10-15 seconds | 25% steering bias | CARLA Simulator |
| **Online PES** | 5000 | 96.08% | 10-15 seconds | 25% steering bias | CARLA Simulator |

**Result:** PES learns model corrections **during driving** with no pre-training on actual vehicle dynamics. Scales gracefully from 5 neurons (89% reduction) to 5000 neurons (96% reduction).

**Breakthrough:** Eliminates system identification bottleneck - learns corrections in real-time instead of months of calibration.

---

## 6. Key Strengths of PES

### Why PES Became the Standard

**Prescribed Error Sensitivity (PES)** emerged as the primary online learning method for adaptive neuromorphic control because:

| Key Strength | Demonstration |
|--------------|---------------|
| **Real-Time Adaptation** | No offline training—learns during operation |
| **Fast Convergence** | 1-2 seconds (2022 payload) to 10-15 seconds (2024 MPC) |
| **Fault Tolerance** | Handles 25% actuator bias seamlessly (2024) |
| **Scalability** | 5 neurons → 89% error reduction; 5000 neurons → 96% |
| **General Applicability** | Works for any error signal without reformulation |
| **Biological Plausibility** | Local Hebbian-like learning rule |
| **Hardware Efficiency** | Event-driven computation on neuromorphic chips |

