import sys
import time
import os

# 添加项目根目录到Python路径，确保可以导入项目模块
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from src.robot_driver.motor_driver.DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable
from src.robot_driver.motor_driver.dm_can_driver import DMMotorDriver

# ===== 用户配置区域 =====
# 修改以下参数以适应您的需求
PORT = 'COM15'           # 串口号，例如 'COM15'
CURRENT_ID = 1        # 当前电机ID
NEW_ID = 13            # 新电机ID
MOTOR_TYPE = DM_Motor_Type.DM4310  # 电机类型，默认为DM4310
# =====================

def change_motor_id():
    """
    连接指定ID的电机并修改其ID
    """
    print(f"=== 达妙电机ID修改工具 ===")
    print(f"正在连接串口 {PORT}...")
    try:
        # 创建电机驱动实例
        driver = DMMotorDriver(port=PORT)
        
        # 添加电机
        print(f"正在添加电机 ID: {hex(CURRENT_ID)}...")
        driver.add_motor(CURRENT_ID, MOTOR_TYPE)
        
        # 使能电机
        print(f"正在使能电机...")
        driver.enable_motor(CURRENT_ID)
        time.sleep(0.5)  # 等待电机使能完成
        
        # 获取电机对象
        motor = driver._get_motor_or_raise(CURRENT_ID)
        
        # 修改电机ID
        print(f"正在将电机ID从 {hex(CURRENT_ID)} 修改为 {hex(NEW_ID)}...")
        success = driver.motor_control.change_motor_param(motor, DM_variable.ESC_ID, NEW_ID)
        
        if success:
            print(f"电机ID修改成功！新ID: {hex(NEW_ID)}")
        else:
            print(f"电机ID修改失败！")
        
        # 失能电机并关闭连接
        driver.disable_motor(CURRENT_ID)
        driver.close()
        
        return success
    
    except Exception as e:
        print(f"发生错误: {e}")
        return False

if __name__ == "__main__":
    change_motor_id()