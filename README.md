**=== To launch/run ===**
**Terminal A** (launching ros):
cd ~/team_say_alyssa
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aps-robot-dog-stabilizer --symlink-install
source /home/pi/team_say_alyssa/install/setup.bash
ros2 launch aps_project aps.launch.py

**Terminal B** (stand mode):
cd ~/team_say_alyssa/aps-robot-dog-stabilizer
source /opt/ros/jazzy/setup.bash
source /home/pi/team_say_alyssa/install/setup.bash
(To run only,) python3 posture_stabalization.py
(To log data,) python3 posture_stabalization.py 2>&1 | tee p_mode_log.txt
                                                          (change file name)

**Terminal C** (additional if gait mode):
cd ~/team_say_alyssa/aps-robot-dog-stabilizer
source /opt/ros/jazzy/setup.bash
source /home/pi/team_say_alyssa/install/setup.bash
python3 gait_node.py

*n.b. When switching modes between Stand/Gait, first kill C -> B, and restart with B -> C.*

**=== To plot ===**
grep "^.*CSV," p_mode_log.txt | sed 's/.*CSV,//' > p_mode_data.csv 
              (change file name)                  (change file name)
python3 plot_compare.py 



