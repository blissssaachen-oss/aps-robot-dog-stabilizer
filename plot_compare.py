import numpy as np
import matplotlib.pyplot as plt

# === LOAD CSV FILES ============ 
PREFIX = 'log_walk/sim_1' # TODO: CHANGE PREFIX
p   = np.loadtxt(f'{PREFIX}_p.csv', delimiter=',')
adr = np.loadtxt(f'{PREFIX}_adrc.csv', delimiter=',')

# ===============================
# FIG 1: P vs ADRC comparison
# "which one keeps roll/pitch closer to 0"
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

ax1.plot(p[:,0], p[:,1], 'b', label='P roll')
ax1.plot(adr[:,0], adr[:,1], 'r', label='ADRC roll')
ax1.set_ylabel('degrees')
ax1.set_title('Roll')
ax1.legend()

ax2.plot(p[:,0], p[:,2], 'b', label='P pitch')
ax2.plot(adr[:,0], adr[:,2], 'r', label='ADRC pitch')
ax2.set_ylabel('degrees')
ax2.set_title('Pitch')
ax2.legend()

plt.tight_layout()
plt.savefig(f'{PREFIX}_comp.png')

# ===============================
# FIG 2: ADRC ESO performance (z1, z2, z3)
# "how accurate is ESO"
# z1 should track actual roll
# z2 should look lika deriv of z1
# z3 should oscillate at gait frequency and stabilize in amplitude
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9))

ax1.plot(adr[:,0], adr[:,1], 'b',     label='actual roll (IMU)')
ax1.plot(adr[:,0], adr[:,5], 'r--',   label='z1 (ESO estimate)')
ax1.set_ylabel('degrees'); ax1.set_title('z1 vs Actual Roll — ESO Tracking Accuracy'); ax1.legend()

ax2.plot(adr[:,0], adr[:,6], 'r', label='z2 (velocity estimate)')
ax2.set_ylabel('deg/s'); ax2.set_title('z2 — Angular Velocity Estimate'); ax2.legend()

ax3.plot(adr[:,0], adr[:,7], 'r', label='z3 (disturbance estimate)')
ax3.set_ylabel('rad/s²'); ax3.set_title('z3 — ESO Disturbance Estimate'); ax3.legend()

plt.tight_layout()
plt.savefig(f'{PREFIX}_ESO.png')

# ===============================
# FIG 3: ESO tracking error + control effort comparison
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

# ESO residual error — how well z1 tracks actual roll
ax1.plot(adr[:,0], adr[:,1] - adr[:,5], 'r', label='roll - z1 (ESO error)')
ax1.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax1.set_ylabel('degrees'); ax1.set_title('ESO Residual Error (roll - z1) — should shrink to ~0'); ax1.legend()

# Control effort comparison — lower effort for same result = better controller
ax2.plot(p[:,0],   p[:,3],   'b', label='P u_roll')
ax2.plot(adr[:,0], adr[:,3], 'r', label='ADRC u_roll')
ax2.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax2.set_ylabel('degrees'); ax2.set_title('Control Effort u_roll — lower = more efficient'); ax2.legend()

plt.tight_layout()
plt.savefig(f'{PREFIX}_analysis.png')

