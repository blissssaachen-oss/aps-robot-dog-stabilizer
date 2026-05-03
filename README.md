--Terminal commands--

**Terminal A** (launching ros):
cd ~/team_say_alyssa
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aps-robot-dog-stabilizer --symlink-install
source /home/pi/team_say_alyssa/install/setup.bash
ros2 launch aps_project aps.launch.py

**Terminal B** (python file):
cd ~/team_say_alyssa/aps-robot-dog-stabilizer
source /opt/ros/jazzy/setup.bash
source /home/pi/team_say_alyssa/install/setup.bash
python3 posture_stabalization.py 
