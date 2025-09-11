import serial
import sys
import os
from time import sleep

# 添加项目根目录到Python路径，确保可以导入项目模块
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from src.robot_driver.motor_driver.DM_CAN import Motor, MotorControl, DM_Motor_Type
from src.robot_driver.motor_driver.dm_can_driver import DMMotorDriver


# --- 用户配置 ---
SERIAL_PORT = 'COM15'   # <-- 修改这里
BAUDRATE = 961200       # SDK默认的波特率
MOTOR_ID_TO_SET = 1     # <-- 修改这里
MOTOR_TYPE = DM_Motor_Type.DM4310
# -----------------


def scan_for_motors(port, baud, target_id=None):
    """
    扫描串口上的电机ID。
    如果指定 target_id，则只检查该ID。
    """
    found_ids = []
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"错误：无法打开串口 {port}。\n详细信息: {e}")
        return []

    controller = MotorControl(ser)

    id_range = [target_id] if target_id is not None else range(1, 17)

    try:
        for test_id in id_range:
            motor = Motor(MotorType=DM_Motor_Type.DM4310, SlaveID=test_id, MasterID=0)
            controller.motors_map.clear()
            controller.addMotor(motor)

            initial_pos = motor.getPosition()
            initial_vel = motor.getVelocity()
            initial_tau = motor.getTorque()

            controller.refresh_motor_status(motor)
            sleep(0.05)
            controller.recv()

            if (motor.getPosition() != initial_pos or
                motor.getVelocity() != initial_vel or
                motor.getTorque() != initial_tau):
                found_ids.append(test_id)
    except Exception as e:
        print(f"扫描错误: {e}")
    finally:
        if controller.serial_.is_open:
            controller.serial_.close()

    return found_ids


def set_zero_position_for_motor(port, baud, motor_id, motor_type):
    """
    将指定ID的电机当前位置设置为其新的零点。
    """
    print("--- 达妙电机设置零点工具 ---")
    print(f"准备将电机 ID: {motor_id} 的当前位置设置为新的零点。")

    # 先扫描确认目标ID是否存在
    print("\n正在检查电机是否在线...")
    online_ids = scan_for_motors(port, baud, target_id=motor_id)
    if motor_id not in online_ids:
        print(f"错误：电机 ID {motor_id} 未在线，请检查连接。")
        return

    print(f"电机 ID {motor_id} 已确认在线。")

    confirm = input("您确定要继续吗？ (y/n): ")
    if confirm.lower() != 'y':
        print("操作已取消。")
        return

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"错误：无法打开串口 {port}。\n详细信息: {e}")
        return

    controller = MotorControl(ser)
    motor = Motor(MotorType=motor_type, SlaveID=motor_id, MasterID=0)
    controller.addMotor(motor)

    try:
        print(f"\n正在向电机 ID {motor_id} 发送设置零点命令...")
        controller.set_zero_position(motor)
        sleep(0.2)
    except Exception as e:
        print(f"操作错误: {e}")
    finally:
        if controller.serial_.is_open:
            controller.serial_.close()
            print("串口已关闭。")


if __name__ == "__main__":
    set_zero_position_for_motor(SERIAL_PORT, BAUDRATE, MOTOR_ID_TO_SET, MOTOR_TYPE)
