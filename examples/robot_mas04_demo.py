

import sys
from pathlib import Path

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.core.robot_driver_mas04 import RobotDriverMas04
import math
import time

# --- 配置 ---
SERIAL_PORT = "COM15"  # 修改为你的实际串口

robot = None
try:
    # 创建机器人实例，初始化和使能电机
    robot = RobotDriverMas04(port=SERIAL_PORT)
    robot.switch_all_mode_mit()

    print("\n--- 复位所有关节到初始位置 (MIT模式) ---")
    robot.home_joints_mit_mode()
    time.sleep(2)

    print("\n--- 进入站立姿态 (MIT模式) ---")
    robot.move_all_mit_mode(
        pos=[
            0, -0.4, -0.8,  # LF
            0, -0.4, -0.8,  # RF
            0, -0.4, -0.8,  # LH
            0, -0.4, -0.8,  # RH
        ]
    )

    # 持续监控机器人状态
    print("\n--- 开始监控所有关节状态 (按 Ctrl+C 停止) ---")
    while True:
        # 获取所有关节的反馈
        joint_states = robot.get_all_feedback()

        # 格式化打印
        print("=" * 80)
        for i in range(0, 12, 3):
            row_str = ""
            for j in range(3):
                joint_id = robot.JOINT_CONFIGS[i + j].id
                state = joint_states.get(joint_id)
                if state:
                    pos_deg = math.degrees(state["position"])
                    row_str += f"ID {hex(joint_id):<4s}: {pos_deg:7.2f}° | "
                else:
                    row_str += f"ID {hex(joint_id):<4s}: [No Data] | "
            print(row_str)

        time.sleep(1)  # 每秒刷新一次

except KeyboardInterrupt:
    print("\n程序被用户中断。")
except Exception as e:
    print(f"\n程序发生错误: {e}")
finally:
    # 无论发生什么，都确保机器人被安全关闭
    if robot:
        robot.shutdown()
