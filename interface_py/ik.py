import numpy as np
import sys, termios, tty, time, math
from startouchclass import SingleArm

# ================== 初始化机械臂 ==================
arm_controller = SingleArm(can_interface_="can0", enable_fd_=False)

# ================== 参数 ==================
POS_STEP = 0.005          # 2 mm
RPY_STEP = math.radians(1.0)  # 1 deg

pos = np.array([0.25, 0.0, 0.175], dtype=float)
euler = np.array([0.0, 0.0, 0.0], dtype=float)

# ================== 终端按键工具 ==================
def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def print_help():
    print("""
========== Cartesian Keyboard Control ==========
Position:
  w/s : +X / -X
  a/d : +Y / -Y
  r/f : +Z / -Z

Orientation (RPY):
  i/k : +Roll / -Roll
  j/l : +Pitch / -Pitch
  u/o : +Yaw / -Yaw

Other:
  space : print current pose
  q     : quit
===============================================
""")

# ================== 主循环 ==================
print_help()
arm_controller.set_end_effector_pose_euler_raw(pos, euler)

while True:
    key = getch()

    updated = False

    # -------- Position --------
    if key == 'w':
        pos[0] += POS_STEP
        updated = True
    elif key == 's':
        pos[0] -= POS_STEP
        updated = True
    elif key == 'a':
        pos[1] += POS_STEP
        updated = True
    elif key == 'd':
        pos[1] -= POS_STEP
        updated = True
    elif key == 'r':
        pos[2] += POS_STEP
        updated = True
    elif key == 'f':
        pos[2] -= POS_STEP
        updated = True

    # -------- Orientation --------
    elif key == 'i':
        euler[0] += RPY_STEP
        updated = True
    elif key == 'k':
        euler[0] -= RPY_STEP
        updated = True
    elif key == 'j':
        euler[1] += RPY_STEP
        updated = True
    elif key == 'l':
        euler[1] -= RPY_STEP
        updated = True
    elif key == 'u':
        euler[2] += RPY_STEP
        updated = True
    elif key == 'o':
        euler[2] -= RPY_STEP
        updated = True

    # -------- Other --------
    elif key == ' ':
        print(f"pos = {pos}, euler(rpy) = {euler}")
    elif key == 'q':
        print("Exit.")
        arm_controller.go_home()
        break

    # -------- Send command --------
    if updated:
        st=time.time()
        arm_controller.set_end_effector_pose_euler_raw(pos, euler)
        print("",time.time()-st)
