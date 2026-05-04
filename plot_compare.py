import numpy as np
import matplotlib.pyplot as plt

# === IMU CALIBRATION ===========
ROLL_OFFSET  = 1.147  # degrees — measured avg resting roll
PITCH_OFFSET = -0.611 # degrees — measured avg resting pitch


# === LOAD CSV FILES ============ 
PREFIX = 'log_walk/sim_3' # TODO: CHANGE PREFIX
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
# roll subplot: to see how ADRC tracks actual roll
# whether leading or lagging / whether smoother
# slightly leading would mean it's predictive, smoother shows ESO filtering effect
ax1b = ax1.twinx()
ax1b.plot(adr[:,0], adr[:,3], 'r--', alpha=0.5, label='ADRC u_roll')
ax1b.set_ylabel('u_roll (degrees)', color='r')
ax1b.tick_params(axis='y', labelcolor='r')
ax1.set_title(f'Roll  |  P RMSE={p_roll_rmse:.2f}°   ADRC RMSE={adrc_roll_rmse:.2f}°')
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax1b.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2)

ax2.plot(p[:,0],   p_pitch,   'b', label='P pitch')
ax2.plot(adr[:,0], adr_pitch, 'r', label='ADRC pitch')
ax2.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax2.set_ylabel('degrees')
ax2.set_title(f'Pitch  |  P RMSE={p_pitch_rmse:.2f}°   ADRC RMSE={adrc_pitch_rmse:.2f}°')
ax2.legend()

plt.tight_layout()
plt.savefig(f'{PREFIX}_comp.png')

# ===============================
# FIG 2: ADRC internal ESO accuracy (z1, z2, z3)
# "how accurate is ESO"
# z1 should track actual roll
# z2 should look lika deriv of z1
# z3 should oscillate at gait frequency and stabilize in amplitude
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9))

ax1.plot(adr[:,0], adr_roll, 'b',     label='actual roll (IMU)')
ax1.plot(adr[:,0], adr[:,5] - ROLL_OFFSET, 'r--',   label='z1 (ESO estimate)')
ax1.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax1.set_ylabel('degrees'); ax1.set_title('z1 vs Actual Roll'); ax1.legend()

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
fig, ax = plt.subplots(1, 1, figsize=(10, 4))

ax.plot(p[:,0],   np.abs(p[:,3]),   'b-',  label='P u_roll')
ax.plot(p[:,0],   np.abs(p[:,4]),   'b--', label='P u_pitch')
ax.plot(adr[:,0], np.abs(adr[:,3]), 'r-',  label='ADRC u_roll')
ax.plot(adr[:,0], np.abs(adr[:,4]), 'r--', label='ADRC u_pitch')

ax.axhline(0, color='k', linewidth=0.5, linestyle='--')
ax.set_ylabel('degrees')
ax.set_title('Control Effort (absolute)  |  solid=roll  dashed=pitch  blue=P  red=ADRC')
ax.legend()

plt.tight_layout()
plt.savefig(f'{PREFIX}_effort.png')


# ===============================
# FIG 4: Gait disturbance reference vs body response
# "what the gait is doing vs what the body actually does"

try:
    gait = np.loadtxt(f'{PREFIX}_adrc.csv', delimiter=',')
    gait[:,0] -= gait[0,0]
    gait = gait[gait[:,0] <= T]

    # convert FR foot z deviation to equivalent body roll disturbance
    roll_lever   = 0.085  # meters (half lateral stance width)
    fr_z_dev     = gait[:,1] - (-0.14)  # deviation from home z
    disturbance_deg = np.degrees(np.arctan2(fr_z_dev, roll_lever))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

    # body response vs disturbance input
    ax1.plot(gait[:,0], disturbance_deg, 'g',  alpha=0.5, label='gait disturbance (estimated)')
    ax1.plot(p[:,0],    p_roll,          'b',  label='P roll')
    ax1.plot(adr[:,0],  adr_roll,        'r',  label='ADRC roll')
    ax1.axhline(0, color='k', linewidth=0.5, linestyle='--')
    ax1.set_ylabel('degrees')
    ax1.set_title('Disturbance Input vs Body Response — smaller gap = better rejection')
    ax1.legend()

    # FR and BL z trajectories — shows gait pattern
    ax2.plot(gait[:,0], gait[:,1], 'b', label='FR z')
    ax2.plot(gait[:,0], gait[:,2], 'r', label='BL z')
    ax2.axhline(-0.14, color='k', linewidth=0.5, linestyle='--', label='home z')
    ax2.set_ylabel('meters')
    ax2.set_title('Foot Z Trajectories (Pair A: FR + BL)')
    ax2.legend()

    plt.tight_layout()
    plt.savefig(f'{PREFIX}_ref.png')

except FileNotFoundError:
    print("No gait CSV found — skipping FIG 4")

