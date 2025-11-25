# HRC_Capstone




python src/main.py



# real robot
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.191 launch_rviz:=false


ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_[t](README.md)ype:=ur3e onrobot_type:=rg2 moveit_joint_limits_file:=/home/dan/ws_moveit/install/ur3_joints_limited/share/ur3_joints_limited/config/ur3/joint_limits.yaml

# simulation
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  use_fake_hardware:=true \
  launch_rviz:=false

ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  moveit_joint_limits_file:=/home/dan/ws_moveit/install/ur3_joints_limited/share/ur3_joints_limited/config/ur3/joint_limits.yaml

cd ~/HRC_Capstone
source venv/bin/activate
python src/main.py


ip addr show
# Remove the current IP
sudo ip addr del 192.168.0.193/24 dev enx00e04c682c73

# Add the new IP
sudo ip addr add 192.168.0.101/24 dev enx00e04c682c73

ip addr show enxa0cec8efd19b

