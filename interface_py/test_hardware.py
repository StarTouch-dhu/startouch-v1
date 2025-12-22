import numpy as np
import sys, termios, tty, time, math
from typing import Tuple
# 创建 ArmController 对象，使用默认构造函数
import os
from startouchclass import SingleArm

arm_controller = SingleArm(can_interface_ = "can0",enable_fd_ = False)

##查看是否正常初始化，电机使能


