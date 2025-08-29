# HRC_Capstone

cd ~/HRC_Capstone
source venv/bin/activate



git add .
git commit -m ""
git push

python3 src/llm_framework/llm_manager.py 



To run we need to setup fake robot that sends joint angles etc: 
    ros2 launch ur_robot_driver ur3.launch.py robot_ip:=xxx.xxx.xxx.xxx use_fake_hardware:=true launch_rviz:=false 

The launch 
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true 
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true moveit_joint_limits_file:=/home/dan/ws_moveit/install/ur3_joints_limited/share/ur3_joints_limited/config/ur3/joint_limits.yaml

Launch with Gazebo
    ros2 launch gazebo_ros gazebo.launch.py world:=/home/dan/ros2_ws/src/custom_worlds/worlds/ur3_workspace.world
    
    ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py \
    ur_type:=ur3 \
    launch_rviz:=true \
    moveit_joint_limits_file:=/home/dan/ws_moveit/install/ur3_joints_limited/share/ur3_joints_limited/config/ur3/joint_limits.yaml



python src/robot_control/motion_planner.py


python src/main.py
