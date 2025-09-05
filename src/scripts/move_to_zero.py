import math
import time
import serial
from typing import Dict, Optional
import sys
import os
# 添加项目根目录到Python路径，确保可以导入项目模块
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from src.robot_driver.motor_driver.DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type


class DMMotorDriver:
    def __init__(self, port: str, baudrate: int = 921600):
        print(f"正在打开串口 {port}，波特率 {baudrate}...")
        self.serial_device = serial.Serial(port, baudrate, timeout=0.1)
        self.motor_control = MotorControl(self.serial_device)
        self.motors: Dict[int, Motor] = {}
        print("串口已打开，电机控制器已初始化。")

    def add_motor(self, motor_id: int, motor_type: DM_Motor_Type) -> bool:
        if motor_id in self.motors:
            print(f"警告: 电机ID {hex(motor_id)} 已存在，跳过添加。")
            return False
        
        host_id = motor_id + 0x10
        motor = Motor(motor_type, motor_id, host_id)
        self.motor_control.addMotor(motor)

        # --- 检查是否在线 ---
        self.motor_control.refresh_motor_status(motor)
        time.sleep(0.05)
        self.motor_control.recv()
        if motor.getPosition() == 0.0 and motor.getVelocity() == 0.0 and motor.getTorque() == 0.0:
            print(f"电机 {hex(motor_id)} 未响应，跳过添加。")
            return False

        self.motors[motor_id] = motor
        print(f"电机 {hex(motor_id)} ({motor_type.name}) 已添加并在线。")
        return True

    def enable_motor(self, motor_id: int):
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.enable(motor)
        print(f"电机 {hex(motor_id)} 已使能。")

    def disable_motor(self, motor_id: int):
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.disable(motor)
        print(f"电机 {hex(motor_id)} 已失能。")

    def enable_all(self):
        for motor_id in self.motors:
            self.enable_motor(motor_id)

    def disable_all(self):
        for motor_id in self.motors:
            self.disable_motor(motor_id)

    # --- 控制模式 ---
    def move_mit_mode(self, motor_id: int, pos_rad: float, vel_rad_s: float, kp: float, kd: float, torque_nm: float):
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.switchControlMode(motor, Control_Type.MIT)
        self.motor_control.controlMIT(motor, kp, kd, pos_rad, vel_rad_s, torque_nm)

    def ramp_to_position(self, motor_id: int, target_pos_rad: float, kp=20.0, kd=0.1, torque=0.0, step_time=0.02, max_step=0.02):
        """
        平滑移动到目标位置，避免猛然转动。
        :param motor_id: 电机ID
        :param target_pos_rad: 目标角度 (弧度)
        :param step_time: 插值的时间间隔 (秒)
        :param max_step: 每步的最大位置增量 (弧度)，用于限制速度
        """
        feedback = self.get_feedback(motor_id)
        if not feedback:
            print(f"电机 {hex(motor_id)} 无反馈，无法执行 ramp。")
            return

        current_pos = feedback['position']
        diff = target_pos_rad - current_pos
        steps = int(abs(diff) / max_step) + 1
        print(f"电机 {hex(motor_id)} 从 {math.degrees(current_pos):.1f}° 平滑移动到 {math.degrees(target_pos_rad):.1f}°，分 {steps} 步。")

        for i in range(steps):
            intermediate_pos = current_pos + (i + 1) / steps * diff
            self.move_mit_mode(motor_id, intermediate_pos, 0.0, kp, kd, torque)
            time.sleep(step_time)

    # --- 反馈 ---
    def get_feedback(self, motor_id: int) -> Optional[Dict[str, float]]:
        motor = self._get_motor_or_raise(motor_id)
        try:
            self.motor_control.refresh_motor_status(motor)
            time.sleep(0.01)
            self.motor_control.recv()
            return {
                "position": motor.getPosition(),
                "velocity": motor.getVelocity(),
                "torque": motor.getTorque(),
            }
        except Exception as e:
            print(f"获取电机 {hex(motor_id)} 反馈时出错: {e}")
            return None

    def close(self):
        print("正在关闭电机驱动器...")
        if self.motors:
            self.disable_all()
            time.sleep(0.1)
        if self.serial_device and self.serial_device.is_open:
            self.serial_device.close()
            print("串口已关闭。")

    def _get_motor_or_raise(self, motor_id: int) -> Motor:
        motor = self.motors.get(motor_id)
        if not motor:
            raise KeyError(f"错误: 电机ID {hex(motor_id)} 未注册到驱动器。")
        return motor

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# ======================================================================
# 使用示例
# ======================================================================
if __name__ == "__main__":
    PORT = 'COM15'
    MOTOR_IDS_TO_TEST = [1,2,3,4,5,6,7,8,9,10,11,12]  # 要测试的电机ID列表
    MOTOR_TYPE_4310 = DM_Motor_Type.DM4310

    try:
        with DMMotorDriver(port=PORT) as driver:
            for motor_id in MOTOR_IDS_TO_TEST:
                driver.add_motor(motor_id, MOTOR_TYPE_4310)

            driver.enable_all()
            time.sleep(1)

            print("\n--- 测试平滑控制 ---")
            for motor_id in MOTOR_IDS_TO_TEST:
                target_pos = math.radians(0.0)
                driver.ramp_to_position(motor_id, target_pos)

            print("\n--- 开始监控反馈 ---")
            while True:
                fb_str = ""
                for motor_id in MOTOR_IDS_TO_TEST:
                    feedback = driver.get_feedback(motor_id)
                    if feedback:
                        pos_deg = math.degrees(feedback['position'])
                        vel_rads = feedback['velocity']
                        tor_nm = feedback['torque']
                        fb_str += f"ID {hex(motor_id)} -> Pos:{pos_deg:7.2f}° Vel:{vel_rads:6.2f} Tor:{tor_nm:5.2f} Nm | "
                    else:
                        fb_str += f"ID {hex(motor_id)} -> [获取失败] | "
                print(f"\r{fb_str}", end="", flush=True)
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n程序被用户中断。")
    except Exception as e:
        print(f"\n程序错误: {e}")
