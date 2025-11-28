from startouchclass import SingleArm
import time

# 创建机械臂连接  连接接口为"can0"
arm_controller = SingleArm(can_interface_ = "can0")


# #获得当前关节位置
# joint_positions = arm_controller.get_joint_positions()
# print("joint_positions",joint_positions,type(joint_positions))
# #获得当前关节速度
# joint_velocities = arm_controller.get_joint_velocities()
# print("joint_velocities",joint_velocities,type(joint_velocities))
# #获得当前关节力矩
# joint_torques = arm_controller.get_joint_torques()
# print("joint_torques",joint_torques,type(joint_torques))

#回到home点
# arm_controller.go_home()
# 结束机械臂控制，删除指定机械臂对象
try:
    print("重力补偿已启动，按ESC键退出...")
    while True:
        # 执行重力补偿
        arm_controller.gravity_compensation()
            
        # 短暂延时，避免过度占用CPU
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("\n程序被用户中断")
except Exception as e:
    print(f"发生错误: {e}")
finally:
    print("重力补偿循环结束")

arm_controller.cleanup()

