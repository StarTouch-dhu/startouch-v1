#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹回放脚本
从 merged_trajectory.txt 读取轨迹数据并回放到机械臂
"""

import sys
import os
import time
import numpy as np
from pathlib import Path

# 添加 startouch-v1/interface_py 到路径
current_dir = Path(__file__).parent
interface_py_dir = current_dir / "startouch-v1" / "interface_py"
if interface_py_dir.exists():
    sys.path.insert(0, str(interface_py_dir))
else:
    print(f"警告: 找不到 interface_py 目录: {interface_py_dir}")

try:
    from startouchclass import SingleArm
    # 尝试导入工具函数
    try:
        from startouchclass import quaternion_to_euler, euler_to_quaternion
    except ImportError:
        # 如果无法导入，则自己定义（从 startouchclass.py 复制）
        def quaternion_to_euler(quat: np.ndarray) -> np.ndarray:
            """将四元数转换为欧拉角（roll, pitch, yaw）"""
            w, x, y, z = quat
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = np.pi / 2 * np.sign(sinp)
            else:
                pitch = np.arcsin(sinp)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)
            return np.array([roll, pitch, yaw])
        
        def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
            """将欧拉角转换为四元数"""
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy
            return np.array([w, x, y, z])
except ImportError as e:
    print(f"错误: 无法导入 startouchclass: {e}")
    print(f"请确保 startouch-v1/interface_py 目录存在且包含 startouchclass.py")
    sys.exit(1)


def quaternion_multiply(q1, q2):
    """
    四元数乘法: q_result = q1 * q2
    参数: q1, q2 为 [w, x, y, z] 格式
    返回: [w, x, y, z] 格式的四元数
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return np.array([w, x, y, z])


def quaternion_rotate_vector(quat, vec):
    """
    使用四元数旋转向量
    参数:
        quat: 四元数 [w, x, y, z]
        vec: 三维向量 [x, y, z]
    返回: 旋转后的向量
    """
    # 将向量转换为四元数 [0, x, y, z]
    vec_quat = np.array([0.0, vec[0], vec[1], vec[2]])
    
    # 计算 q * vec_quat * q^(-1)
    quat_conj = np.array([quat[0], -quat[1], -quat[2], -quat[3]])
    result = quaternion_multiply(quat, quaternion_multiply(vec_quat, quat_conj))
    
    # 返回旋转后的向量部分
    return result[1:4]


def align_trajectory_to_initial_pose(trajectory, initial_pos, initial_euler):
    """
    将轨迹对齐到夹爪的初始位姿
    
    参数:
        trajectory: 轨迹数据列表（以第一个点为原点）
        initial_pos: 夹爪初始位置 [x, y, z] (米)
        initial_euler: 夹爪初始欧拉角 [roll, pitch, yaw] (弧度)
        
    返回:
        aligned_trajectory: 对齐后的轨迹数据
    """
    if len(trajectory) == 0:
        return trajectory
    
    print("\n对齐轨迹到初始位姿...")
    print(f"初始位置: [{initial_pos[0]:.6f}, {initial_pos[1]:.6f}, {initial_pos[2]:.6f}] (米)")
    print(f"初始欧拉角: [{initial_euler[0]:.6f}, {initial_euler[1]:.6f}, {initial_euler[2]:.6f}] (弧度)")
    
    # 将初始欧拉角转换为四元数
    initial_quat = euler_to_quaternion(initial_euler[0], initial_euler[1], initial_euler[2]) # wxyz
    
    # 获取轨迹的第一个点（作为参考原点）
    first_point = trajectory[0]
    ref_pos = first_point['position']
    ref_quat = first_point['quaternion']
    
    print(f"轨迹参考原点位置: [{ref_pos[0]:.6f}, {ref_pos[1]:.6f}, {ref_pos[2]:.6f}] (米)")
    print(f"轨迹参考原点四元数: [{ref_quat[0]:.6f}, {ref_quat[1]:.6f}, {ref_quat[2]:.6f}, {ref_quat[3]:.6f}]")
    
    # 对齐后的轨迹
    aligned_trajectory = []
    
    # 计算从轨迹参考坐标系到世界坐标系的变换
    # 参考四元数的共轭（用于旋转）
    ref_quat_conj = np.array([ref_quat[0], -ref_quat[1], -ref_quat[2], -ref_quat[3]])
    
    for i, point in enumerate(trajectory):
        # 计算相对于参考原点的位移（在轨迹坐标系中）
        relative_pos = point['position'] - ref_pos
        
        # 将相对位移从轨迹坐标系旋转到世界坐标系
        # 使用参考四元数的逆来旋转
        rotated_relative_pos = quaternion_rotate_vector(ref_quat_conj, relative_pos)
        
        # 计算对齐后的位置：初始位置 + 旋转后的相对位移
        aligned_pos = initial_pos + rotated_relative_pos
        
        # 计算对齐后的姿态
        # 1. 计算当前点相对于参考点的旋转：current_quat * ref_quat_conj
        relative_quat = quaternion_multiply(point['quaternion'], ref_quat_conj) #wxyz
        
        # 2. 将相对旋转应用到初始姿态：initial_quat * relative_quat
        aligned_quat = quaternion_multiply(initial_quat, relative_quat) # wxyz
        
        # 归一化四元数（确保是单位四元数）
        norm = np.linalg.norm(aligned_quat)
        if norm > 1e-6:
            aligned_quat = aligned_quat / norm
        else:
            # 如果四元数接近零，使用初始四元数
            aligned_quat = initial_quat.copy()
        
        aligned_trajectory.append({
            'timestamp': point['timestamp'],
            'position': aligned_pos,
            'quaternion': aligned_quat
        })
        
        # 显示第一个和最后一个点的对齐结果
        if i == 0 or i == len(trajectory) - 1:
            aligned_euler = quaternion_to_euler(aligned_quat)
            print(f"\n点 {i+1}:")
            print(f"  原始位置: [{point['position'][0]:.6f}, {point['position'][1]:.6f}, {point['position'][2]:.6f}]")
            print(f"  对齐后位置: [{aligned_pos[0]:.6f}, {aligned_pos[1]:.6f}, {aligned_pos[2]:.6f}]")
            print(f"  对齐后欧拉角: [{aligned_euler[0]:.6f}, {aligned_euler[1]:.6f}, {aligned_euler[2]:.6f}] (弧度)")
    
    print(f"\n✓ 轨迹对齐完成，共 {len(aligned_trajectory)} 个点")
    return aligned_trajectory


# 不对齐姿态，直接使用轨迹的姿态
# def align_trajectory_to_initial_pose(trajectory, initial_pos):
#     """
#     将轨迹对齐到夹爪的初始位置
    
#     参数:
#         trajectory: 轨迹数据列表（以第一个点为原点）
#         initial_pos: 夹爪初始位置 [x, y, z] (米)
        
#     返回:
#         aligned_trajectory: 对齐后的轨迹数据
#     """
#     if len(trajectory) == 0:
#         return trajectory
    
#     print("\n对齐轨迹到初始位置...")
#     print(f"初始位置: [{initial_pos[0]:.6f}, {initial_pos[1]:.6f}, {initial_pos[2]:.6f}] (米)")
    
#     # 获取轨迹的第一个点（作为参考原点）
#     first_point = trajectory[0]
#     ref_pos = first_point['position']
    
#     print(f"轨迹参考原点位置: [{ref_pos[0]:.6f}, {ref_pos[1]:.6f}, {ref_pos[2]:.6f}] (米)")
    
#     # 对齐后的轨迹
#     aligned_trajectory = []
    
#     # 计算从轨迹参考坐标系到世界坐标系的变换
#     # 计算相对于参考原点的位移（在轨迹坐标系中）
#     for i, point in enumerate(trajectory):
#         # 计算相对位移
#         relative_pos = point['position'] - ref_pos
        
#         # 对齐后的目标位置：初始位置 + 相对位移
#         aligned_pos = initial_pos + relative_pos
        
#         # 保持轨迹中的姿态不变
#         aligned_quat = point['quaternion']
        
#         aligned_trajectory.append({
#             'timestamp': point['timestamp'],
#             'position': aligned_pos,
#             'quaternion': aligned_quat
#         })
        
#         # 显示第一个和最后一个点的对齐结果
#         if i == 0 or i == len(trajectory) - 1:
#             print(f"\n点 {i+1}:")
#             print(f"  原始位置: [{point['position'][0]:.6f}, {point['position'][1]:.6f}, {point['position'][2]:.6f}]")
#             print(f"  对齐后位置: [{aligned_pos[0]:.6f}, {aligned_pos[1]:.6f}, {aligned_pos[2]:.6f}]")
    
#     print(f"\n✓ 轨迹对齐完成，共 {len(aligned_trajectory)} 个点")
#     return aligned_trajectory



def load_trajectory(trajectory_file):
    """
    加载轨迹文件
    
    参数:
        trajectory_file: 轨迹文件路径
        
    返回:
        trajectory: 列表，每个元素为 [timestamp, x, y, z, qx, qy, qz, qw]
    """
    trajectory = []
    
    with open(trajectory_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:  # 跳过空行
                continue
            
            parts = line.split()
            if len(parts) != 8:
                print(f"警告: 跳过格式不正确的行: {line}")
                continue
            
            try:
                timestamp = float(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                
                # 四元数格式转换为 [w, x, y, z]
                quat = np.array([qw, qx, qy, qz])  # 注意顺序
                pos = np.array([x, y, z])
                
                trajectory.append({
                    'timestamp': timestamp,
                    'position': pos,
                    'quaternion': quat
                })
            except ValueError as e:
                print(f"警告: 解析行时出错 {line}: {e}")
                continue
    
    print(f"成功加载 {len(trajectory)} 个轨迹点")
    return trajectory


def replay_trajectory(arm, trajectory, use_raw=True, speed_factor=1.0):
    """
    回放轨迹到机械臂
    
    参数:
        arm: SingleArm 实例
        trajectory: 轨迹数据列表
        use_raw: 是否使用原始模式（透传，无规划）。True 使用 raw 模式，False 使用规划模式
        speed_factor: 速度因子，>1.0 加速，<1.0 减速
    """
    if len(trajectory) < 2:
        print("错误: 轨迹点数量不足")
        return
    
    print(f"开始回放轨迹，共 {len(trajectory)} 个点")
    print(f"模式: {'原始模式（透传）' if use_raw else '规划模式'}")
    print(f"速度因子: {speed_factor}x")
    
    # 计算时间间隔
    time_diffs = []
    for i in range(1, len(trajectory)):
        dt = (trajectory[i]['timestamp'] - trajectory[i-1]['timestamp']) / speed_factor
        time_diffs.append(max(0.001, dt))  # 最小间隔 1ms
    
    # 移动到起始位置
    first_point = trajectory[0]
    print(f"移动到起始位置: {first_point['position']}")
    print(f"移动到起始位置: {first_point['quaternion']}")
    if use_raw:
        arm.set_end_effector_pose_quat_raw(
            pos=first_point['position'],
            quat=first_point['quaternion']
        )
        print("2s后开始执行轨迹")
        time.sleep(2)
    else:
        arm.set_end_effector_pose_quat(
            pos=first_point['position'],
            quat=first_point['quaternion'],
            tf=0.5  # 初始移动时间
        )
    
    time.sleep(0.5)  # 等待到达起始位置
    
    # 开始回放
    print("开始回放...")
    start_time = time.time()
    
    try:
        for i in range(1, len(trajectory)):
            point = trajectory[i]
            if use_raw:
                arm.set_end_effector_pose_quat_raw(
                    pos=point['position'],
                    quat=point['quaternion']
                )
            else:
                # 使用规划模式时，根据时间间隔设置 tf
                # tf = time_diffs[i-1] if i-1 < len(time_diffs) else 0.01
                tf = 0.001
                arm.set_end_effector_pose_quat(
                    pos=point['position'],
                    quat=point['quaternion'],
                    tf=tf
                )
            
            # 等待到下一个点的时间
            if i < len(time_diffs):
                time.sleep(time_diffs[i-1])
            
            # 每100个点打印一次进度
            if (i + 1) % 100 == 0:
                elapsed = time.time() - start_time
                progress = (i + 1) / len(trajectory) * 100
                print(f"进度: {progress:.1f}% ({i+1}/{len(trajectory)}), 已用时: {elapsed:.2f}s")
    
    except KeyboardInterrupt:
        print("\n回放被用户中断")
    except Exception as e:
        print(f"回放过程中出错: {e}")
        import traceback
        traceback.print_exc()
    
    elapsed = time.time() - start_time
    print(f"回放完成！总用时: {elapsed:.2f}s")


def main():
    """主函数"""
    # 轨迹文件路径
    # trajectory_file = current_dir / "multi_sessions_20251216_095739" / "session_001" / "Merged_Trajectory" / "merged_trajectory.txt"
    trajectory_file = current_dir /"merged_trajectory.txt"
    
    if not trajectory_file.exists():
        print(f"错误: 轨迹文件不存在: {trajectory_file}")
        return
    
    # 加载轨迹
    print(f"加载轨迹文件: {trajectory_file}")
    trajectory = load_trajectory(trajectory_file)
    
    if len(trajectory) == 0:
        print("错误: 没有有效的轨迹数据")
        return
    
    # 初始化机械臂
    print("初始化机械臂...")
    try:
        arm = SingleArm(can_interface_="can0", gripper=True, enable_fd_=False)
        print("机械臂初始化成功")
    except Exception as e:
        print(f"错误: 机械臂初始化失败: {e}")
        import traceback
        traceback.print_exc()
        return
    
    try:
        # 等待机械臂稳定
        print("\n等待机械臂稳定...")
        time.sleep(1.0)
        
        # 获取夹爪的初始位姿
        print("获取夹爪初始位姿...")
        try:
            initial_pos, initial_euler = arm.get_ee_pose_euler()
            print(f"初始位置 (x, y, z): [{initial_pos[0]:.6f}, {initial_pos[1]:.6f}, {initial_pos[2]:.6f}] (米)")
            print(f"初始欧拉角 (roll, pitch, yaw): [{initial_euler[0]:.6f}, {initial_euler[1]:.6f}, {initial_euler[2]:.6f}] (弧度)")
            print(f"初始欧拉角 (roll, pitch, yaw): [{np.degrees(initial_euler[0]):.2f}, {np.degrees(initial_euler[1]):.2f}, {np.degrees(initial_euler[2]):.2f}] (度)")
        except Exception as e:
            print(f"错误: 获取初始位姿失败: {e}")
            import traceback
            traceback.print_exc()
            return
        
        # 将轨迹对齐到初始位姿
        aligned_trajectory = align_trajectory_to_initial_pose(trajectory, initial_pos,initial_euler)
        
        # 询问用户选择回放模式
        print("\n请选择回放模式:")
        print("1. 原始模式（透传，实时回放，推荐）")
        print("2. 规划模式（平滑，但可能较慢）")
        choice = input("请输入选择 (1/2，默认1): ").strip()
        
        use_raw = (choice != '2')
        
        # 询问速度因子
        speed_input = input("请输入速度因子 (默认1.0，>1.0加速，<1.0减速): ").strip()
        try:
            speed_factor = float(speed_input) if speed_input else 1.0
        except ValueError:
            speed_factor = 1.0
            print("使用默认速度因子: 1.0")
        
        # 确认开始
        print("\n准备开始回放...")
        input("按 Enter 键开始回放（或 Ctrl+C 取消）")
        
        # 回放对齐后的轨迹
        replay_trajectory(arm, aligned_trajectory, use_raw=use_raw, speed_factor=speed_factor)
        
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        print("清理资源...")
        try:
            arm.cleanup()
        except:
            pass
        print("程序结束")


if __name__ == "__main__":
    main()

