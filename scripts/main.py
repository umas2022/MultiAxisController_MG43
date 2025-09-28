

import sys
from pathlib import Path
import math
import time
import numpy as np

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.core.robot_driver_mas04 import RobotDriverMas04
from src.models.policy_runner_pt import PolicyRunnerPT
from src.drivers.imu_driver.imu_driver_isaac import IMUDriver

# --- 配置 ---
MOTOR_PORT = "COM15"  
IMU_PORT = "COM13"
MODEL_PATH = "..\src\models\pt\model_2025-09-11_17-12-00.pt"


robot = None
try:


    # 创建机器人实例，初始化和使能电机
    robot = RobotDriverMas04(port=MOTOR_PORT)
    robot.switch_all_mode_mit()

    # 创建IMU实例并启动
    imu = IMUDriver(port=IMU_PORT, baud=921600)
    imu.start()

    print("\n--- 复位所有关节到初始位置 (MIT模式) ---")
    robot.home_joints_mit_mode()
    time.sleep(2)

    print("\n--- 进入站立姿态 (MIT模式) ---")
    robot.move_all_mit_mode_smooth(
        target_positions=robot.DEFAULT_POS,
        velocity=3,
        min_step_deg=1
    )
    time.sleep(2)

    print("\n--- 启动 RL 控制循环 ---")
    # 加载策略模型
    policy = PolicyRunner(
        model_path=MODEL_PATH,
        num_actions=12,
        num_obs=3 + 3 + 3 + 3 + 12 + 12 + 12,
        default_joint_pos=robot.DEFAULT_POS_ISAAC,
        action_scale=0.25,
    )

    last_actions = [0]*12
    target_pos = np.array(robot.DEFAULT_POS_ISAAC)
    while True:
        # 获取IMU数据
        base_linear_velocity,base_angular_velocity, projected_gravity= imu.get_isaaclab_data()

        # 获取机器人状态
        joint_positions, joint_velocities, joint_torques = robot.get_all_feedback_isaac()

        velocity_commands = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # 传入虚假速度
        base_linear_velocity = velocity_commands
        joint_positions = target_pos

        # --- 构造观测向量（示例: 全零） ---
        obs_vector = np.concatenate([
            base_linear_velocity,
            base_angular_velocity,
            projected_gravity,
            velocity_commands,
            joint_positions,
            joint_velocities,
            last_actions
        ], axis=0)

        # --- 获取电机目标角 ---
        last_actions,target_pos = policy.infer_action(obs_vector)


        print("\n==============================================\n")
        print(f"base_linear_velocity (m/s): {[round(v.item(),3) for v in base_linear_velocity]}")
        print(f"base_angular_velocity (rad/s): {[round(v.item(),3) for v in base_angular_velocity]}")
        print(f"projected_gravity (m/s^2): {[round(v.item(),3) for v in projected_gravity]}")
        print(f"velocity_commands (m/s): {[round(v.item(),3) for v in velocity_commands]}")
        print(f"joint_positions (rad): {[round(v.item(),3) for v in joint_positions]}")
        print(f"joint_velocities (rad/s): {[round(v.item(),3) for v in joint_velocities]}")
        print(f"last_actions (rad): {[round(v.item(),3) for v in last_actions]}")
        print(f"target_pos (rad):{[round(v.item(),3) for v in target_pos]}")



        robot_order = []
        for i in [0,3,6,9,1,4,7,10,2,5,8,11]:
            robot_order.append(target_pos[i])
        # robot.move_all_mit_mode(robot_order)

        time.sleep(0.1)
        wait_here = input("press enter to continue ...")


except KeyboardInterrupt:
    print("\n程序被用户中断。")
except Exception as e:
    print(f"\n程序发生错误: {e}")
finally:
    # 无论发生什么，都确保机器人被安全关闭
    if robot:
        robot.shutdown()
    
    # 关闭IMU驱动
    if 'imu' in locals() and imu is not None:
        imu.stop()
        print("IMU驱动已关闭")
    
    # 强制退出所有线程
    import os
    os._exit(0)
