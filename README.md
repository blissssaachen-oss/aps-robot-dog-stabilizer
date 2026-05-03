# ====== To launch/run ====================
**Terminal A** (launching ros):
cd ~/team_say_alyssa
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aps-robot-dog-stabilizer --symlink-install
source /home/pi/team_say_alyssa/install/setup.bash
ros2 launch aps_project aps.launch.py

**Terminal B** (only if gait mode):
cd ~/team_say_alyssa/aps-robot-dog-stabilizer
source /opt/ros/jazzy/setup.bash && source /home/pi/team_say_alyssa/install/setup.bash
python3 gait_node.py

**Terminal C**:
cd ~/team_say_alyssa/aps-robot-dog-stabilizer
source /opt/ros/jazzy/setup.bash && source /home/pi/team_say_alyssa/install/setup.bash
(To run only,) python3 posture_stabalization.py
(To log data,) (**TODO: only change PREFIX**)
PREFIX=log_walk/sim_1 
RCUTILS_COLORIZED_OUTPUT=0 python3 posture_stabalization.py 2>&1 | tee ${PREFIX}_p.txt
RCUTILS_COLORIZED_OUTPUT=0 python3 posture_stabalization.py 2>&1 | tee ${PREFIX}_adrc.txt

# *n.b. When done, first kill C -> B.*
# *n.b. When LOGGING,run C first and B with tee (to start logging upon gait launch)*

# ====== To log ===========================
(**TODO: only change PREFIX**)
PREFIX=log_walk/sim_1 
grep "CSV," ${PREFIX}_p.txt | sed 's/.*CSV,//' > ${PREFIX}_p.csv
grep "CSV," ${PREFIX}_adrc.txt | sed 's/.*CSV,//' > ${PREFIX}_adrc.csv

# *after logging both for P -> ADRC, then plot:*
python3 plot_compare.py 

# ====== To export (all) plot ==============
scp -r pi@<pi_ip_address>:~/team_say_alyssa/aps-robot-dog-stabilizer/log_walk .




