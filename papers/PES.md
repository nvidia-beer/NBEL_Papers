## Prescribed Error Sensitivity (PES) Online Learning

### Notation Table

\footnotesize

| Notation | Description |
| :-- | :-- |
| $\Delta \mathbf{w}$ | Update to decoder/output weight vector |
| $\kappa$ | PES learning rate |
| $\mathbf{e}(t)$ | Error signal (target output minus current output) at time $t$ |
| $\mathbf{a}(t)$ | Population neural activity (filtered spikes or rates) at $t$ |
| $W$ | Decoder matrix (for multi-dimensional outputs) |
| $\mathbf{y}(t)$ | Decoded network output at time $t$ |
| $\mathbf{y}_{target}(t)$ | Desired/supervised output at time $t$ |

\normalsize

### Mathematical Formulation

Prescribed Error Sensitivity is an online learning rule for adapting output/decoder weights to minimize the error between a network’s output and a target signal:

$$
\Delta \mathbf{w} = \kappa \, \mathbf{e}(t) \, \mathbf{a}(t)
$$

For multi-dimensional outputs:

$$
\Delta W = \kappa \, \mathbf{e}(t) \, \mathbf{a}(t)^\top
$$

where the network output is

$$
\mathbf{y}(t) = W^\top \mathbf{a}(t)
$$

and

$$
\mathbf{e}(t) = \mathbf{y}_{\text{target}}(t) - \mathbf{y}(t)
$$

Filtering of activity and error (with time constants of 50–200 ms) is standard to ensure stability and smooth learning.

### Workflow

- Compute neural activity ($\mathbf{a}(t)$).
- Linearly decode the output ($\mathbf{y}(t)$).
- Calculate error ($\mathbf{e}(t)$).
- Adapt decoder weights with $\Delta W$.

### Stability and Scaling

- Learning rate ($\kappa$) must ensure $1 - \kappa \|\mathbf{a}\|^2 > -1$ to avoid instability.
- Increasing neuron number improves error convergence and final performance, as demonstrated in all major NBEL experiments.
- Filtering input and error signals further enhances robustness.

### Implementation Insights from NBEL Research

- **Learning rate:** Empirically chosen for each hardware and task; fast rates adapt rapidly but risk instability, while slower rates ensure robust gradual learning .
- **Neural count:** Higher neuron counts yield faster convergence and finer error minimization (500+ neurons deliver >95% reduction in tested tasks) .
- **Task flexibility:** Error signal can target diverse outputs, from robot joint angles to vehicle path deviations [2021–2024].
- **Adaptation:** PES-driven systems robustly recover from disturbances within seconds, as verified in arm and vehicle control studies.
- **Hardware:** PES is compatible and proven effective on neuromorphic hardware (Intel Loihi, Nengo), providing energy-efficient and real-time learning .

PES thus operates as a core, validated, adaptable online learning mechanism for neuromorphic and real-time control in spiking neural networks—supporting rapid, stable, and hardware-friendly adaptation across a range of advanced robotic and control tasks.

