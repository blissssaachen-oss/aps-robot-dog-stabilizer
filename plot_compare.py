import numpy as np
import matplotlib.pyplot as plt

# === IMU CALIBRATION ===========
ROLL_OFFSET  = 1.147  # degrees — measured avg resting roll
PITCH_OFFSET = -0.611 # degrees — measured avg resting pitch

# === LOAD CSV FILES ============ 
PREFIX = 'log_walk/sim_1' # TODO: CHANGE PREFIX
p   = np.loadtxt(f'{PREFIX}_p.csv', delimiter=',')
adr = np.loadtxt(f'{PREFIX}_adrc.csv', delimiter=',')
# normalize time (start from 0 + trim to same duration)
p[:,0]   -= p[0,0]
adr[:,0] -= adr[0,0]
T = min(p[-1,0], adr[-1,0])
p   = p[p[:,0]     <= T]
adr = adr[adr[:,0] <= T]
# apply offset correction
p_roll   = p[:,1]   - ROLL_OFFSET
p_pitch  = p[:,2]   - PITCH_OFFSET
adr_roll = adr[:,1] - ROLL_OFFSET
adr_pitch= adr[:,2] - PITCH_OFFSET

# ===============================
# RMSE
def rmse(x): return np.sqrt(np.mean(x**2))
p_roll_rmse    = rmse(p_roll)
p_pitch_rmse   = rmse(p_pitch)
adrc_roll_rmse = rmse(adr_roll)
adrc_pitch_rmse= rmse(adr_pitch)
print(f"P    roll RMSE: {p_roll_rmse:.3f}°  pitch RMSE: {p_pitch_rmse:.3f}°")
print(f"ADRC roll RMSE: {adrc_roll_rmse:.3f}°  pitch RMSE: {adrc_pitch_rmse:.3f}°")

# ===============================
# FIG 1: P vs ADRC comparison
# "which one keeps roll/pitch closer to 0"
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
ax1.plot(p[:,0],   p_roll,   'b', label='P roll')
ax1.plot(adr[:,0], adr_roll, 'r', label='ADRC roll')
ax1.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax1.set_ylabel('degrees')
ax1.set_title(f'Roll  |  P RMSE={p_roll_rmse:.2f}°   ADRC RMSE={adrc_roll_rmse:.2f}°')
ax1.legend()

ax2.plot(p[:,0],   p_pitch,   'b', label='P pitch')
ax2.plot(adr[:,0], adr_pitch, 'r', label='ADRC pitch')
ax2.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax2.set_ylabel('degrees')
ax2.set_title(f'Pitch  |  P RMSE={p_pitch_rmse:.2f}°   ADRC RMSE={adrc_pitch_rmse:.2f}°')
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

ax1.plot(adr[:,0], adr_roll, 'b',     label='actual roll (IMU)')
ax1.plot(adr[:,0], adr[:,5] - ROLL_OFFSET, 'r--',   label='z1 (ESO estimate)')
ax1.plot(adr[:,0], adr_roll - (adr[:,5] - ROLL_OFFSET), 'g:', label='residual error') # remove maybe
ax1.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax1.set_ylabel('degrees'); ax1.set_title('z1 vs Actual Roll + Residual error'); ax1.legend()

ax2.plot(adr[:,0], adr[:,6], 'r', label='z2 (velocity estimate)')
ax2.set_ylabel('deg/s'); ax2.set_title('z2 — Angular Velocity Estimate'); ax2.legend()

ax3.plot(adr[:,0], adr[:,7], 'r', label='z3 (disturbance estimate)')
ax3.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax3.set_ylabel('rad/s²'); ax3.set_title('z3 — ESO Disturbance Estimate'); ax3.legend()

plt.tight_layout()
plt.savefig(f'{PREFIX}_ESO.png')

# ===============================
# FIG 3: Control effort comparison
# "lower effort for same result = better controller"
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

ax2.plot(p[:,0],   p[:,3],   'b', label='P u_roll')
ax2.plot(adr[:,0], adr[:,3], 'r', label='ADRC u_roll')
ax2.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax2.set_ylabel('degrees'); ax2.set_title('Control Effort u_roll'); ax2.legend()

ax2.plot(p[:,0],   p[:,4],   'b', label='P u_pitch')
ax2.plot(adr[:,0], adr[:,4], 'r', label='ADRC u_pitch')
ax2.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax2.set_ylabel('degrees'); ax2.set_title('Control Effort u_pitch'); ax2.legend()

plt.tight_layout()
plt.savefig(f'{PREFIX}_effort.png')

