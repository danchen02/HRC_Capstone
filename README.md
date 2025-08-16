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

python src/robot_control/motion_planner.py


python src/main.py
