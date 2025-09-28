'''
记录函数执行时间
静止状态下：
    move_all_mit_mode 函数平均执行时间0.001848s
    get_all_feedback 函数平均执行时间0.001960s
    move_all_mit_mode+get_all_feedback 函数平均执行时间0.003712s
'''

import os
import sys
import time
import math

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from src.core.robot_driver_mas04 import RobotDriverMas04

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


    start_time = time.perf_counter()
    fb = robot.get_all_feedback()
    print(time.perf_counter() - start_time)

    time_record = []
    for i in range(100):
        fb_all = robot.get_all_feedback()
        pos_list = []
        for id in fb_all:
            pos_list.append(fb_all[id]["position"])

        start_time = time.perf_counter()
        robot.move_all_mit_mode(pos=pos_list)
        robot.get_all_feedback()
        function_time = time.perf_counter() - start_time

        time_record.append(function_time)
        print(f"获取时间 {i+1:03d}: {function_time:.6f} 秒")

    print(f"平均获取时间: {sum(time_record)/len(time_record):.6f} 秒")





except KeyboardInterrupt:
    print("\n程序被用户中断。")
except Exception as e:
    print(f"\n程序发生错误: {e}")
finally:
    # 无论发生什么，都确保机器人被安全关闭
    if robot:
        robot.shutdown()
