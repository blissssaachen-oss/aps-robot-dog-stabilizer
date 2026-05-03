import numpy as np

# grep "CSV," log_walk/stand_test.txt | sed 's/.*CSV,//' > log_walk/stand_test.csv

stand = np.loadtxt('log_walk/stand_test_1.csv', delimiter=',')
# columns: time, roll, pitch, u_roll, u_pitch, z1, z2, z3

# average offset
print(f"ROLL_OFFSET  = {np.mean(stand[:,1]):.3f}")
print(f"PITCH_OFFSET = {np.mean(stand[:,2]):.3f}")
# how jittery IMU is
print(f"roll std: {np.std(stand[:,1]):.3f}°")
print(f"pitch std: {np.std(stand[:,2]):.3f}°")

"""
std: Useful for RMSE interpret later
If RMSE from a gait run is similar to the stand std,
then the controller is essentially rejecting disturbance 
down to the noise floor, which would be a strong result.
"""
