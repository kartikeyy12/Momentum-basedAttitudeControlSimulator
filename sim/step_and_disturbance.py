"""
ADCS-1 Phase 3: Time-domain simulation
Reference step response and disturbance-torque rejection, PD vs Lead.
"""
import numpy as np
import matplotlib.pyplot as plt
import control as ct

# --- Plant + controllers (from Phase 1 & 2) ---
J_b = 2e-3
K_p, K_d = 0.016327, 0.008
G = ct.tf([1], [J_b, 0, 0])
C_pd = ct.tf([K_d, K_p], [1])

PM_t, wc_t = 60, 6.0
phi = np.radians(PM_t)
alpha = (1 - np.sin(phi)) / (1 + np.sin(phi))
sa = np.sqrt(alpha)
z, p = wc_t * sa, wc_t / sa
K_lead = J_b * wc_t**2 / sa
C_lead = ct.tf([K_lead, K_lead*z], [1, p])

controllers = {"PD": C_pd, "Lead": C_lead}
t = np.linspace(0, 5, 3000)
ref_deg = 10.0        # 10-degree pointing maneuver
d_torque = 0.002      # N*m constant disturbance torque

# ================= Reference step response =================
print("--- Step Response (10 deg maneuver) ---")
plt.figure()
for name, C in controllers.items():
    T_ref = ct.feedback(G * C, 1)                 # ref -> theta
    tout, yout = ct.step_response(T_ref, T=t)
    plt.plot(tout, yout * ref_deg, label=name)
    info = ct.step_info(T_ref, T=t)
    print(f"{name:5s} rise={info['RiseTime']:.3f}s  settle={info['SettlingTime']:.3f}s  "
          f"overshoot={info['Overshoot']:.1f}%")
plt.axhline(ref_deg, color='gray', linestyle='--', linewidth=0.7, label='target')
plt.xlabel("Time [s]"); plt.ylabel("Angle [deg]")
plt.title(f"Step Response: {ref_deg} deg Pointing Maneuver")
plt.legend(); plt.grid(True)
plt.savefig("../docs/step_response.png", dpi=150)

# ================= Disturbance-torque rejection =================
print("\n--- Disturbance Rejection (2 mN*m constant torque) ---")
plt.figure()
for name, C in controllers.items():
    T_dist = ct.feedback(G, C)                    # disturbance torque -> theta
    tout, yout = ct.step_response(d_torque * T_dist, T=t)
    plt.plot(tout, np.degrees(yout), label=name)
    ss_err = np.degrees(yout[-1])
    peak = np.degrees(np.max(np.abs(yout)))
    print(f"{name:5s} steady-state error={ss_err:.3f} deg   peak={peak:.3f} deg")
plt.xlabel("Time [s]"); plt.ylabel("Angle error [deg]")
plt.title(f"Disturbance Rejection: {d_torque*1000:.0f} mN*m constant torque")
plt.legend(); plt.grid(True)
plt.savefig("../docs/disturbance_rejection.png", dpi=150)

plt.show()

# ================= 2-DOF fix: reference prefilter =================
# T(s) = C(s)G(s)/(1+C(s)G(s)) inherits C(s)'s zero in its numerator.
# That zero (close to the poles) is what inflated overshoot above.
# A prefilter F(s) = z_c/(s+z_c) cancels it algebraically, touching
# ONLY the reference path -- stability, margins, and disturbance
# rejection are all governed by L(s)=C(s)G(s), which is unchanged.

z_pd = K_p / K_d      # PD's closed-loop zero
F_pd = ct.tf([z_pd], [1, z_pd])

z_lead_val = z        # Lead's zero, from earlier in this script
F_lead = ct.tf([z_lead_val], [1, z_lead_val])

T_ref_pd_raw   = ct.feedback(G * C_pd, 1)
T_ref_lead_raw = ct.feedback(G * C_lead, 1)
T_ref_pd_filt   = F_pd * T_ref_pd_raw
T_ref_lead_filt = F_lead * T_ref_lead_raw

print("\n--- Step Response: raw vs prefiltered (2-DOF) ---")
plt.figure()
for name, sysT in [("PD raw", T_ref_pd_raw), ("PD + prefilter", T_ref_pd_filt),
                    ("Lead raw", T_ref_lead_raw), ("Lead + prefilter", T_ref_lead_filt)]:
    tout, yout = ct.step_response(sysT, T=t)
    plt.plot(tout, yout * ref_deg, label=name)
    info = ct.step_info(sysT, T=t)
    print(f"{name:16s} rise={info['RiseTime']:.3f}s  settle={info['SettlingTime']:.3f}s  "
          f"overshoot={info['Overshoot']:.1f}%")
plt.axhline(ref_deg, color='gray', linestyle='--', linewidth=0.7, label='target')
plt.xlabel("Time [s]"); plt.ylabel("Angle [deg]")
plt.title("Step Response: Raw vs Prefiltered (2-DOF) Reference Tracking")
plt.legend(); plt.grid(True)
plt.savefig("../docs/step_response_prefiltered.png", dpi=150)
plt.show()
