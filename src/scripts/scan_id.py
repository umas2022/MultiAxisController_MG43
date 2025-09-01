
import sys
import os

# 添加项目根目录到Python路径，确保可以导入项目模块
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from src.robot_driver.motor_driver.DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable
from src.robot_driver.motor_driver.dm_can_driver import DMMotorDriver

import serial
from time import sleep

# --- 用户配置 ---
# 请根据您的实际情况修改这里的串口号
# Windows: 'COM3', 'COM4', ...
# Linux: '/dev/ttyUSB0', '/dev/ttyACM0', ...
# macOS: '/dev/cu.usbserial-XXXX', ...
SERIAL_PORT = 'COM15'  # <-- 修改这里
BAUDRATE = 961200     # SDK默认的波特率，通常不需要修改

# 设置要扫描的电机ID范围
# 大多数情况下，ID不会超过16或32，可以根据需要调整
MIN_ID = 1
MAX_ID = 16

# 每次测试ID之间的等待时间（秒）
# 如果扫描不稳定，可以适当增加这个值
SCAN_DELAY = 0.05
# --- 配置结束 ---


def scan_for_motors(port, baud, min_id, max_id):
    """
    扫描指定串口上的达妙电机ID。

    :param port: 串口号
    :param baud: 波特率
    :param min_id: 扫描的起始ID
    :param max_id: 扫描的结束ID
    :return: 找到的在线电机ID列表
    """
    found_ids = []
    
    try:
        # 初始化串口
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"错误：无法打开串口 {port}。")
        print(f"详细信息: {e}")
        print("请检查：")
        print("1. 串口号是否正确。")
        print("2. 电机或USB-CAN模块是否已连接并通电。")
        print("3. 是否有其他程序占用了该串口。")
        return []

    # 创建电机控制器实例
    controller = MotorControl(ser)
    
    print(f"正在扫描串口 {port}，ID范围：{min_id} 到 {max_id}...")
    
    try:
        for test_id in range(min_id, max_id + 1):
            # 打印进度
            print(f"  正在测试 ID: {test_id} ... ", end='', flush=True)
            
            # 1. 为当前测试的ID创建一个临时电机对象
            #    MotorType在这里不关键，因为我们只检查是否有响应，而不关心具体数值的解析
            motor_to_test = Motor(MotorType=DM_Motor_Type.DM4310, SlaveID=test_id, MasterID=0)

            # 2. 清空控制器内部的电机映射表，确保每次只监听一个ID
            controller.motors_map.clear()
            
            # 3. 将当前要测试的电机添加到映射表中
            controller.addMotor(motor_to_test)
            
            # 记录发送命令前的状态（初始应为0）
            initial_pos = motor_to_test.getPosition()
            initial_vel = motor_to_test.getVelocity()
            initial_tau = motor_to_test.getTorque()

            # 4. 发送一个状态刷新请求，这是一个无害的查询命令
            controller.refresh_motor_status(motor_to_test)
            
            # 5. 等待一小段时间，给电机响应和数据传输留出时间
            sleep(SCAN_DELAY)
            
            # 6. 接收并处理串口数据
            controller.recv()
            
            # 7. 检查电机状态是否被更新
            #    如果电机响应，SDK的recv()会更新对象的状态值。
            #    只要有一个值不再是初始的0.0，就说明收到了来自该ID的数据包。
            if (motor_to_test.getPosition() != initial_pos or
                motor_to_test.getVelocity() != initial_vel or
                motor_to_test.getTorque() != initial_tau):
                
                found_ids.append(test_id)
                print(f"在线！")
            else:
                print(f"无响应。")

    except Exception as e:
        print(f"\n扫描过程中发生意外错误: {e}")
    finally:
        # 确保在程序结束时关闭串口
        if controller.serial_.is_open:
            controller.serial_.close()
            print("\n串口已关闭。")
            
    return found_ids


if __name__ == "__main__":
    online_motors = scan_for_motors(SERIAL_PORT, BAUDRATE, MIN_ID, MAX_ID)
    
    print("\n--- 扫描完成 ---")
    if online_motors:
        print(f"在总线上找到以下电机ID: {online_motors}")
    else:
        print("未在指定范围内找到任何在线电机。")
        print("请检查：")
        print("  - 电机是否已正确连接并上电。")
        print("  - USB转CAN/TTL模块的接线（TX/RX）是否正确。")
        print("  - 扫描的ID范围（MIN_ID, MAX_ID）是否包含了您的电机ID。")
        print("  - 电机固件是否支持 `refresh_motor_status` 命令。")

