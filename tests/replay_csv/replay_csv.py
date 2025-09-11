import csv
import os
import sys
import time
import math

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from src.core.robot_driver_mas04 import Controller12

# 替换为你的CSV文件路径
filename = 'positions.csv'

SERIAL_PORT = "COM15"  # 修改为你的实际串口

robot = None
# 1. 创建机器人实例，这会自动初始化和使能电机
robot = Controller12(port=SERIAL_PORT)

# 2. 发送高级动作指令
robot.home_joints_pv_mode()
time.sleep(2)
robot.switch_all_mode_mit()

# 记录开始时间
total_start_time = time.time()
row_count = 0

with open(filename, 'r') as file:
    reader = csv.reader(file)
    headers = next(reader)  # 读取标题行
    
    # 找出所有position相关的列索引
    pos_columns = [i for i, col in enumerate(headers) if col.startswith('pos_')]
    pos_headers = [headers[i] for i in pos_columns]
    
    print("Position columns:", pos_headers)
    print("-" * 100)
    
    # 遍历每一行数据
    for row_num, row in enumerate(reader, 1):
        # 记录单行开始时间
        row_start_time = time.time()
        
        # 提取position列的数据并单独赋值
        LF_HAA = float(row[pos_columns[0]])
        LH_HAA = float(row[pos_columns[1]])
        RF_HAA = float(row[pos_columns[2]])
        RH_HAA = float(row[pos_columns[3]])
        LF_HFE = float(row[pos_columns[4]])
        LH_HFE = float(row[pos_columns[5]])
        RF_HFE = float(row[pos_columns[6]])
        RH_HFE = float(row[pos_columns[7]])
        LF_KFE = float(row[pos_columns[8]])
        LH_KFE = float(row[pos_columns[9]])
        RF_KFE = float(row[pos_columns[10]])
        RH_KFE = float(row[pos_columns[11]])
        
        # 注意：这里需要修正，应该是12个位置值，而不是[0]
        pos_list = [
            LF_HAA, LF_HFE, LF_KFE,    # 左前腿
            RF_HAA, RF_HFE, RF_KFE,    # 右前腿  
            LH_HAA, LH_HFE, LH_KFE,    # 左后腿
            RH_HAA, RH_HFE, RH_KFE     # 右后腿
        ]

        print(f"Row {row_num}: {[f'{x:.3f}' for x in pos_list]}")
        
        # 发送控制命令
        robot.move_all_mit_mode(pos=pos_list)
        feedback = robot.get_all_joint_feedback()
        print("  Feedback:", [f"{math.degrees(feedback[joint_id]['position']):.2f}°" for joint_id in sorted(feedback.keys())])

        time.sleep(0.02)
        
        # 记录单行结束时间并计算耗时
        row_end_time = time.time()
        row_time = row_end_time - row_start_time
        
        row_count += 1
        

robot.shutdown()

# 记录总结束时间并计算总耗时
total_end_time = time.time()
total_time = total_end_time - total_start_time

print("=" * 60)
print(f"循环完成!")
print(f"总处理行数: {row_count} 行")
print(f"总耗时: {total_time:.3f} 秒")
print(f"平均每行耗时: {total_time/row_count:.3f} 秒" if row_count > 0 else "无数据行")
print(f"平均频率: {row_count/total_time:.2f} Hz" if total_time > 0 and row_count > 0 else "无频率数据")