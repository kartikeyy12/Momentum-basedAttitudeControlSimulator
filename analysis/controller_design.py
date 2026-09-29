"""
ADCS-1: PD and Lead controller design verification
Plant: G(s) = 1 / (J_b * s^2)
"""
import numpy as np
import matplotlib.pyplot as plt
import control as ct

# --- Parameters (from Phase 1 & 2) ---
J_b = 2e-3       # kg*m^2, PLACEHOLDER -- update after bifilar pendulum test
K_p = 0.016327   # N*m/rad
K_d = 0.008      # N*m*s/rad

# --- Build transfer functions ---
G = ct.tf([1], [J_b, 0, 0])       # plant: 1/(J_b s^2)
C = ct.tf([K_d, K_p], [1])        # PD controller: K_d*s + K_p
L = G * C                          # open loop

print("Plant G(s):", G)
print("Controller C(s):", C)

# --- Stability margins ---
gm, pm, wg, wp = ct.margin(L)
print(f"\nPhase margin: {pm:.1f} deg at {wp:.3f} rad/s")
print(f"Gain margin:  {gm:.2f} (infinite for this plant type is normal)")

# --- Bode plot ---
plt.figure()
ct.bode_plot(L, dB=True, deg=True, display_margins=True)
plt.suptitle("Open-Loop Bode Plot with Margins (PD)")
plt.savefig("../docs/bode_plot.png", dpi=150)

# --- Root locus ---
plt.figure()
ct.root_locus(L, grid=True)
plt.title("Root Locus of Compensated Open Loop (PD)")
plt.savefig("../docs/root_locus.png", dpi=150)

# --- Nyquist plot ---
plt.figure()
ct.nyquist_plot(L)
plt.title("Nyquist Plot (PD)")
plt.savefig("../docs/nyquist_plot.png", dpi=150)

# --- Why the D term matters: pole migration as K_d increases ---
Kd_values = np.linspace(0, K_d, 25)
plt.figure()
for kd in Kd_values:
    poles = np.roots([J_b, kd, K_p])
    color = 'red' if kd == 0 else 'blue'
    plt.scatter(poles.real, poles.imag, color=color, s=20)
plt.scatter([], [], color='red', label='K_d = 0 (undamped)')
plt.scatter([], [], color='blue', label='K_d > 0 (damped)')
plt.axvline(0, color='gray', linewidth=0.5)
plt.axhline(0, color='gray', linewidth=0.5)
plt.xlabel("Real")
plt.ylabel("Imaginary")
plt.title("Closed-Loop Poles as K_d Increases (K_p fixed)")
plt.legend()
plt.grid(True)
plt.savefig("../docs/damping_effect.png", dpi=150)

print("\nPD plots saved to docs/")

# =========================================================
# Lead Compensator (Phase 2b) - the practical version of PD
# =========================================================
PM_target = 60
wc_target = 6.0

phi_max = np.radians(PM_target)
alpha = (1 - np.sin(phi_max)) / (1 + np.sin(phi_max))
sqrt_alpha = np.sqrt(alpha)

z = wc_target * sqrt_alpha
p = wc_target / sqrt_alpha
K_lead = (J_b * wc_target**2) / sqrt_alpha

print(f"\n--- Lead Compensator Design ---")
print(f"alpha = {alpha:.4f}, zero = {z:.3f} rad/s, pole = {p:.3f} rad/s, K = {K_lead:.4f}")

C_lead = ct.tf([K_lead, K_lead*z], [1, p])
L_lead = G * C_lead

gm2, pm2, wg2, wp2 = ct.margin(L_lead)
print(f"Achieved PM: {pm2:.1f} deg at {wp2:.3f} rad/s (target: {PM_target} deg at {wc_target} rad/s)")

plt.figure()
ct.bode_plot([L, L_lead], dB=True, deg=True, display_margins=False)
plt.legend(["PD", "Lead"])
plt.suptitle("Open-Loop Bode: PD vs Lead Compensator")
plt.savefig("../docs/bode_pd_vs_lead.png", dpi=150)

w_hf = 1000
mag_pd_hf = np.abs(K_p + 1j*K_d*w_hf)
mag_lead_hf = np.abs(K_lead*(1j*w_hf + z)/(1j*w_hf + p))
print(f"\nController gain at {w_hf} rad/s (noise band):")
print(f"  PD:   {mag_pd_hf:.2f}  <- grows without bound")
print(f"  Lead: {mag_lead_hf:.2f}  <- bounded, flattens to K = {K_lead:.3f}")

print("\nAll plots saved to docs/")
plt.show()
