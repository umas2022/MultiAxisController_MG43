
# 这个脚本无法永久修改电机id！
# 修改id必须使用串口模式上位机

import sys
import time
import os

from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))


from src.drivers.motor_driver.DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable
from src.drivers.motor_driver.motor_driver import MotorDriver

# ===== 用户配置区域 =====
# 修改以下参数以适应您的需求
PORT = 'COM15'           # 串口号，例如 'COM15'
CURRENT_ID = 3        # 当前电机ID
# =====================

def print_feedback():
    """
    打印电机反馈信息
    """
    print(f"=== 达妙电机ID修改工具 ===")
    print(f"正在连接串口 {PORT}...")
    try:
        # 创建电机驱动实例
        driver = MotorDriver(port=PORT)
        
        # 添加电机
        print(f"正在添加电机 ID: {hex(CURRENT_ID)}...")
        driver.add_motor(CURRENT_ID, DM_Motor_Type.DM4310)
        
        # 使能电机
        print(f"正在使能电机...")
        driver.enable_motor(CURRENT_ID)
        time.sleep(0.5)  # 等待电机使能完成
        
        # 获取并打印反馈
        while True:
            feedback = driver.get_feedback(CURRENT_ID)
            print(f"反馈: {feedback}")
            time.sleep(0.5)
        

    except Exception as e:
        print(f"发生错误: {e}")
        return False

if __name__ == "__main__":
    print_feedback()