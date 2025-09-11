import math
import serial
import time
from typing import Dict, Optional, Tuple

from src.drivers.motor_driver.DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type


class MotorDriver:
    """
    达妙电机(DM)的驱动类，用于通过CAN转串口模块管理和控制多个电机。
    这个类封装了底层的SDK，提供了一个更高级、面向对象的接口。
    """

    def __init__(self, port: str, baudrate: int = 921600):
        """
        初始化驱动器，打开串口并准备与电机通信。
        
        :param port: 串口号，例如 'COM15' 或 '/dev/ttyUSB0'。
        :param baudrate: 波特率，默认为921600。
        """
        print(f"正在打开串口 {port}，波特率 {baudrate}...")
        self.serial_device = serial.Serial(port, baudrate, timeout=0.1)
        self.motor_control = MotorControl(self.serial_device)
        self.motors: Dict[int, Motor] = {}
        print("串口已打开，电机控制器已初始化。")

    def add_motor(self, motor_id: int, motor_type: DM_Motor_Type) -> bool:
        """
        向驱动器注册一个新的电机。
        
        :param motor_id: 电机的CAN ID。
        :param motor_type: 电机的型号，例如 DM_Motor_Type.DM4310。
        :return: 如果成功添加则返回True，如果ID已存在则返回False。
        """
        if motor_id in self.motors:
            print(f"警告: 电机ID {hex(motor_id)} 已存在，跳过添加。")
            return False
        
        host_id = motor_id + 0x10
        motor = Motor(motor_type, motor_id, host_id)
        self.motors[motor_id] = motor
        self.motor_control.addMotor(motor)
        return True

    def enable_motor(self, motor_id: int):
        """使能指定的电机。"""
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.enable(motor)
        print(f"电机 {hex(motor_id)} 已使能。")

    def disable_motor(self, motor_id: int):
        """失能指定的电机。"""
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.disable(motor)
        print(f"电机 {hex(motor_id)} 已失能。")
        
    def enable_all(self):
        """使能所有已添加的电机。"""
        for motor_id in self.motors:
            self.enable_motor(motor_id)

    def disable_all(self):
        """失能所有已添加的电机。"""
        for motor_id in self.motors:
            self.disable_motor(motor_id)

    # --- 控制模式切换 ---
    def switch_mode_mit(self, motor_id: int):
        """将指定电机切换到MIT控制模式。"""
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.switchControlMode(motor, Control_Type.MIT)

    def switch_mode_pos_vel(self, motor_id: int):
        """将指定电机切换到位置速度控制模式。"""
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.switchControlMode(motor, Control_Type.POS_VEL)

    def switch_all_mode_mit(self):
        """将所有电机切换到MIT控制模式。"""
        for motor_id in self.motors:
            self.switch_mode_mit(motor_id)

    def switch_all_mode_pos_vel(self):
        """将所有电机切换到位置速度控制模式。"""
        for motor_id in self.motors:
            self.switch_mode_pos_vel(motor_id)

    # --- 控制指令 ---
    def move_mit_mode(self, motor_id: int, pos_rad: float, vel_rad_s: float, kp: float, kd: float, torque_nm: float):
        """
        设置指定电机为MIT控制模式并发送指令。
        这是最常用的模式，可同时控制位置、速度和前馈力矩（阻抗控制）。
        Total_Torque = [ Kp * (Pos_target - Pos_current) ] + [ Kd * (Vel_target - Vel_current) ] + [ Torque_feedforward ]
        """
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.controlMIT(motor, kp, kd, pos_rad, vel_rad_s, torque_nm)

    def move_pos_vel_mode(self, motor_id: int, pos_rad: float, vel_rad_s: float):
        """
        设置指定电机为位置速度控制模式并发送指令。
        该模式下只能同时控制位置和速度，无法直接控制力矩。
        """
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.control_Pos_Vel(motor, pos_rad, vel_rad_s)
    

    # --- 反馈与关闭 ---

    def get_feedback(self, motor_id: int) -> Optional[Dict[str, float]]:
        """
        获取指定电机的最新反馈值（位置、速度、力矩）。
        此方法内部完成了“请求-响应”的完整流程。
        """
        motor = self._get_motor_or_raise(motor_id)
        
        try:
            # 步骤 1: 发送状态请求指令
            self.motor_control.refresh_motor_status(motor)
            # 步骤 2: 短暂等待电机响应
            # time.sleep(0.002) # 稍微缩短延时以提高潜在频率
            # 步骤 3: 接收并解析响应数据
            self.motor_control.recv()
            
            # 步骤 4: 返回更新后的数据
            return {
                "position": motor.getPosition(),
                "velocity": motor.getVelocity(),
                "torque": motor.getTorque(),
            }
        except Exception as e:
            print(f"获取电机 {hex(motor_id)} 反馈时出错: {e}")
            return None

    def close(self):
        """安全地失能所有电机并关闭串口连接。"""
        print("正在关闭电机驱动器...")
        if self.motors:
            self.disable_all()
            time.sleep(0.1)
        if self.serial_device and self.serial_device.is_open:
            self.serial_device.close()
            print("串口已关闭。")

    def _get_motor_or_raise(self, motor_id: int) -> Motor:
        """内部辅助函数，获取电机对象或在不存在时抛出异常。"""
        motor = self.motors.get(motor_id)
        if not motor:
            raise KeyError(f"错误: 电机ID {hex(motor_id)} 未注册到驱动器。")
        return motor

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

