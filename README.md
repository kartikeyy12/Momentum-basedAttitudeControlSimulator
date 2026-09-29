# Momentum-based Attitude Control Simulator (ADCS-1)

EC300 Control Systems — Group 20

**Team:** Kartikey Tiwari, Kathan Kuchekar, Mayuresh Nalavade

## Overview
A single-axis attitude control system for a simulated CubeSat, using an
internal reaction wheel for torque-based pointing control — no thrusters,
no aerodynamic surfaces. The satellite body is modeled as a double
integrator, derived from conservation of angular momentum between the
body and the reaction wheel. Controller design (PD + Lead compensator)
targets a specified phase margin, validated via Bode, root locus, and
Nyquist analysis, then verified in Python simulation before hardware
integration.

## Progress
- [x] Phase 0 — Specifications
- [x] Phase 1 — Mathematical Modeling
- [x] Phase 2 — PD Controller Design (validated: Bode, root locus, Nyquist)
- [x] Phase 2b — Lead Compensator Design (validated, matches target PM exactly)
- [x] Phase 3 — Python Simulation (step response, disturbance rejection, 2-DOF prefilter)
- [ ] Phase 4 — Hardware Build
- [ ] Phase 5 — Firmware
- [ ] Phase 6 — Kalman Filter / Friction Feedforward / Saturation Demo

## Plant Model
G(s) = 1 / (J_b·s²) — derived from conservation of angular momentum
between the body (J_b) and reaction wheel (J_w).

| Parameter | Value | Status |
|---|---|---|
| J_b (body inertia) | 2×10⁻³ kg·m² | placeholder — pending bifilar pendulum measurement |
| J_w (flywheel inertia) | 3.0×10⁻⁵ kg·m² | computed from 60mm×3mm steel disc |

## Controller Design
| Controller | Parameters | Achieved PM |
|---|---|---|
| PD | K_p=0.01633, K_d=0.008 | 65.2° at 4.41 rad/s |
| Lead | z=1.608, p=22.39, K=0.269 | 60.0° at 6.00 rad/s (exact) |

## Simulation Results
| Design | Rise (s) | Settle (s) | Overshoot | Disturbance SS error |
|---|---|---|---|---|
| PD (raw) | 0.297 | 1.709 | 21.0% | 7.02° |
| PD + prefilter | 0.745 | 2.094 | 4.6% | 7.02° |
| Lead (raw) | 0.197 | 1.614 | 18.8% | 5.94° |
| **Lead + prefilter (final)** | 0.980 | 1.797 | **0.0%** | **5.94°** |

**Key finding:** raw PD/Lead step responses overshoot more than 2nd-order
theory predicts, because the closed-loop reference transfer function
inherits the controller's own zero. Fixed with a 2-DOF reference
prefilter, touching only the reference path — stability margins and
disturbance rejection are unaffected.

## Repo layout
- `analysis/` — plant modeling, PD/Lead design, Bode/root-locus/Nyquist (`controller_design.py`)
- `sim/` — closed-loop simulation, step response, disturbance rejection (`step_and_disturbance.py`)
- `firmware/` — Arduino code
- `hardware/` — BOM, CAD, wiring
- `docs/` — specs, report, presentation, generated plots

## Setup (WSL/Linux)
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
