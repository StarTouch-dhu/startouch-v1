import numpy as np
import sys, termios, tty, time, math
from typing import Tuple
# 创建 ArmController 对象，使用默认构造函数
import os
from startouchclass import SingleArm

arm_controller = SingleArm(can_interface_ = "can0",enable_fd_ = False)

tf=2.0
# q_end = np.array([0.0, 1.57, 1.57, 0.0, 0.0, 0.0] ).reshape(6,1) # 设置目标关节角度
# arm_controller.set_joint(q_end,ctrl_hz = 400.0)
pos = np.array([0.469373,0, 0.425908])
euler = np.array([0.0, 0.0, 0.0])
arm_controller.set_end_effector_pose_euler(pos,euler,tf)
# arm_controller.set_end_effector_pose_euler_posvel(pos,euler,tf =2.0)
pos1, euler = arm_controller.get_ee_pose_euler()
print("current pos",pos1)
print("current euler",euler)

# arm_controller.openGripper()
# time.sleep(2)
# arm_controller.setGripperPosition(0.0)

# pos1, euler = arm_controller.get_ee_pose_euler()
# print("current pos",pos1)
# print("current euler",euler)

# time.sleep(2)

q_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 设置回到起点的位姿
arm_controller.set_joint(q_start, tf)
# arm_controller.set_joint_posvel(q_start, tf)
# arm_controller.closeGripper()
time.sleep(2)

# arm_controller.gravity_compensation()       



