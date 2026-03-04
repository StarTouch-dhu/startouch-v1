import numpy as np
import sys, termios, tty, time, math
from typing import Tuple
# 创建 ArmController 对象，使用默认构造函数
import os
from startouchclass import SingleArm

arm_controller = SingleArm(can_interface_ = "can0",enable_fd_ = False)
time.sleep(4)
q_sol, ok = arm_controller.solve_ik([0.25, 0.0, 0.45], [1.0, 0.0, 0.0, 0.0])
print(q_sol)
time.sleep(4)
##查看是否正常初始化，电机使能


