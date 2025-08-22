#!/usr/bin/env python3
import subprocess
import os

env = os.environ.copy()

# Define el tipo de robot que estamos usando
ROBOT_TYPE = "xarm"
ROBOT_TYPE2 = "xarm"

print("[SCRIPT LAUNCHER] Iniciando xarm2_random_placer.py...")
env["ROS_NAMESPACE"] = "xarm2"
p2 = subprocess.Popen(['rosrun', 'xarm_gazebo', 'xarm2_random_placer.py', f'_robot_type:={ROBOT_TYPE2}'], env=env)
p2.wait()
print("[SCRIPT LAUNCHER] Finalizado zone_checker.py.")

print("[SCRIPT LAUNCHER] Iniciando zone_checker.py...")
env["ROS_NAMESPACE"] = "xarm"
p2 = subprocess.Popen(['rosrun', 'xarm_gazebo', 'zone_checker.py', f'_robot_type:={ROBOT_TYPE}'], env=env)
p2.wait()
print("[SCRIPT LAUNCHER] Finalizado zone_checker.py.")

print("[SCRIPT LAUNCHER] Iniciando color_recognition.py...")
env["ROS_NAMESPACE"] = "xarm"
p3 = subprocess.Popen(['rosrun', 'xarm_gazebo', 'color_recognition.py', f'_robot_type:={ROBOT_TYPE}'], env=env)
p3.wait()
print("[SCRIPT LAUNCHER] Finalizado color_recognition.py.")