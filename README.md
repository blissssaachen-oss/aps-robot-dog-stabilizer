# To launch/run
## Terminal A (launching ros): 
```bash
cd ~/team_say_alyssa 
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aps-robot-dog-stabilizer --symlink-install
source /home/pi/team_say_alyssa/install/setup.bash
ros2 launch aps_project aps.launch.py
```

## Terminal B (only if gait mode):
```bash
cd ~/team_say_alyssa/aps-robot-dog-stabilizer
source /opt/ros/jazzy/setup.bash && source /home/pi/team_say_alyssa/install/setup.bash
python3 gait_node.py
```

## Terminal C
```bash
cd ~/team_say_alyssa/aps-robot-dog-stabilizer
source /opt/ros/jazzy/setup.bash && source /home/pi/team_say_alyssa/install/setup.bash
```
(First select controller in posture_stabalization.py, line 20) \
(To run only,) 
```bash 
python3 posture_stabalization.py 
```
(To log data,) **TODO: only change PREFIX**
```bash
PREFIX=log_walk/sim_1 
RCUTILS_COLORIZED_OUTPUT=0 python3 posture_stabalization.py 2>&1 | tee ${PREFIX}_p.txt
RCUTILS_COLORIZED_OUTPUT=0 python3 posture_stabalization.py 2>&1 | tee ${PREFIX}_adrc.txt
```

*n.b. When done, first kill B -> C.* \
*n.b. When LOGGING,run B first and C with tee (to ensure log starts after gait launch)* \
For comparison test: **repeat Terminals A-B-C all** to ensure same start state

# To log
**TODO: only change PREFIX**
```bash
PREFIX=log_walk/sim_1 
grep "CSV," ${PREFIX}_p.txt | sed 's/.*CSV,//' > ${PREFIX}_p.csv
grep "CSV," ${PREFIX}_adrc.txt | sed 's/.*CSV,//' > ${PREFIX}_adrc.csv
```

*after logging both for P -> ADRC, then plot:*
```bash
python3 plot_compare.py 
```

# To export (all) plot 
```bash
scp -r pi@<pi_ip_address>:~/team_say_alyssa/aps-robot-dog-stabilizer/log_walk .
```

# Stand test for IMU calib 
(in P mode, only do Terminal A, C.)
```bash
RCUTILS_COLORIZED_OUTPUT=0 python3 posture_stabalization.py 2>&1 | tee log_walk/stand_test.txt
grep "CSV," log_walk/stand_test_adrc.txt | sed 's/.*CSV,//' > log_walk/stand_test.csv
python3 imu_calib.py
```

    ROLL_OFFSET  = 1.147
    PITCH_OFFSET = -0.611
    roll std: 0.008°
    pitch std: 0.012°