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
from src.drivers.elrs_driver.elrs_driver import ElrsReceiver
from src.drivers.ik_driver.ik_driver import CombinedMotionController

# --- 配置 ---
MOTOR_PORT = "COM3"
ELRS_PORT = "COM6"


robot = None
try:

    # 创建机器人实例，初始化和使能电机
    robot = RobotDriverMas04(port=MOTOR_PORT)
    robot.switch_all_mode_mit()

    # 创建ELRS接收机实例并启动
    elrs = ElrsReceiver(serial_port=ELRS_PORT, baud_rate=420000)

    print("\n--- 复位所有关节到初始位置 (MIT模式) ---")
    robot.home_joints_mit_mode()
    time.sleep(2)

    print("\n--- 进入站立姿态 (MIT模式) ---")
    robot.move_all_mit_mode_smooth(
        target_positions=robot.DEFAULT_POS, velocity=3, min_step_deg=1
    )
    time.sleep(2)

    print("\n--- 启动 IK 控制循环 ---")

    controller = CombinedMotionController()
    while True:
        # 输入目标方向
        linear_x,linear_y,angular_z = elrs.get_channel_xyz()
        # print(f"线速度X: {linear_x:.2f}, 线速度Y: {linear_y:.2f}, 角速度Z: {angular_z:.2f}")
        
        motor_angles = controller.get_motor_angles(linear_x, linear_y, angular_z)
        controller.update(linear_x, linear_y, angular_z)

        # wait_here = input("按回车继续")
        robot.move_all_mit_mode(motor_angles)



except KeyboardInterrupt:
    print("\n程序被用户中断。")
except Exception as e:
    print(f"\n程序发生错误: {e}")
finally:
    # 无论发生什么，都确保机器人被安全关闭
    if robot:
        robot.shutdown()

    # 关闭ELRS接收机
    if "elrs" in locals() and elrs is not None:
        elrs.close()
        print("ELRS接收机已关闭")

    # 强制退出所有线程
    import os

    os._exit(0)
