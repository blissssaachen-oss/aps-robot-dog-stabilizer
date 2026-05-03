--How to run--

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
python3 posture_stabalization.py 


**Terminal C** (additional if gait mode):
cd ~/team_say_alyssa/aps-robot-dog-stabilizer
source /opt/ros/jazzy/setup.bash
source /home/pi/team_say_alyssa/install/setup.bash
python3 gait_node.py

n.b. When switching modes between Stand - Gait, 
first kill and restart **Terminal B**.