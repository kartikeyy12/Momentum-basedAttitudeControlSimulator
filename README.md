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
- [x] Phase 2 — Analytical Controller Design (PD)
- [ ] Phase 2b — Lead Compensator
- [ ] Phase 3 — Python Simulation
- [ ] Phase 4 — Hardware Build
- [ ] Phase 5 — Firmware
- [ ] Phase 6 — Kalman Filter / Friction Feedforward / Saturation Demo

## Plant Model
G(s) = 1 / (J_b·s²)  — derived from conservation of angular momentum
between the body (J_b) and reaction wheel (J_w).

## Current Design Parameters
| Parameter | Value | Status |
|---|---|---|
| J_b (body inertia) | 2×10⁻³ kg·m² | placeholder — pending bifilar pendulum measurement |
| J_w (flywheel inertia) | 3.0×10⁻⁵ kg·m² | computed from 60mm×3mm steel disc |
| K_p (PD proportional gain) | 0.0163 N·m/rad | for ζ=0.7, t_s=2s |
| K_d (PD derivative gain) | 0.008 N·m·s/rad | for ζ=0.7, t_s=2s |
| Target phase margin | ~65° | |

## Repo layout
- `analysis/` — plant modeling, controller design, Bode/root-locus/Nyquist scripts
- `sim/` — closed-loop simulation (step response, disturbance rejection)
- `firmware/` — Arduino code
- `hardware/` — BOM, CAD, wiring
- `docs/` — specs, report, presentation

## Setup (WSL/Linux)
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
