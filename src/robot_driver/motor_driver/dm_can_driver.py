import math
import serial
import time
from typing import Dict, Optional, Tuple

from src.robot_driver.motor_driver.DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type


class DMMotorDriver:
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
        print(f"电机 {hex(motor_id)} ({motor_type.name}) 已添加。")
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

    # --- 控制模式封装 ---

    def set_mit_mode(self, motor_id: int, pos_rad: float, vel_rad_s: float, kp: float, kd: float, torque_nm: float):
        """
        设置指定电机为MIT控制模式并发送指令。
        这是最常用的模式，可同时控制位置、速度和前馈力矩（阻抗控制）。
        Total_Torque = [ Kp * (Pos_target - Pos_current) ] + [ Kd * (Vel_target - Vel_current) ] + [ Torque_feedforward ]
        """
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.switchControlMode(motor, Control_Type.MIT)
        self.motor_control.controlMIT(motor, kp, kd, pos_rad, vel_rad_s, torque_nm)
    
    def set_pos_vel_mode(self, motor_id: int, pos_rad: float, vel_rad_s: float):
        """
        设置指定电机为位置速度模式（传统的PD控制）。
        电机将尝试同时达到目标位置和速度。
        """
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.switchControlMode(motor, Control_Type.POS_VEL)
        self.motor_control.control_Pos_Vel(motor, pos_rad, vel_rad_s)

    def set_vel_mode(self, motor_id: int, vel_rad_s: float):
        """
        设置指定电机为速度模式。
        电机将尝试以指定的速度匀速转动。
        """
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.switchControlMode(motor, Control_Type.VEL)
        self.motor_control.control_Vel(motor, vel_rad_s)

    def set_torque_pos_mode(self, motor_id: int, pos_rad: float, vel_des: float, i_des: float):
        """

        设置指定电机为力位混合模式。
        控制位置的同时，施加一个指定的电流（力矩）。
        注意: vel_des 和 i_des 的单位和范围需要参考SDK文档。
        """
        motor = self._get_motor_or_raise(motor_id)
        self.motor_control.switchControlMode(motor, Control_Type.Torque_Pos)
        self.motor_control.control_pos_force(motor, pos_rad, vel_des, i_des)

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
            time.sleep(0.005) # 稍微缩短延时以提高潜在频率
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

# ======================================================================
#                            使用示例
# ======================================================================
if __name__ == "__main__":
    # --- 配置 ---
    PORT = 'COM15'  # 修改为你的串口
    MOTOR_IDS_TO_TEST = [1] 
    MOTOR_TYPE_4310 = DM_Motor_Type.DM4310

    try:
        # 使用 'with' 语句自动管理资源的打开和关闭，非常安全
        with DMMotorDriver(port=PORT) as driver:
            # 1. 添加并使能所有需要的电机
            for motor_id in MOTOR_IDS_TO_TEST:
                driver.add_motor(motor_id, MOTOR_TYPE_4310)
            
            driver.enable_all()
            print(f"\n已使能电机: {[hex(mid) for mid in MOTOR_IDS_TO_TEST]}")
            time.sleep(1) # 等待电机使能完成

            print("\n--- 开始测试电机控制模式 ---")

            # ======================================================================
            # --- 示例1: MIT模式 (默认启用) ---
            # 目标: 让两个电机像弹簧一样运动到相反的位置。
            #       电机0x0C到+90度，电机0x0D到-90度。
            # ======================================================================
            print("\n测试: MIT模式")
            kp, kd, torque = 5.0, 0.5, 0.0
            vel_target = 0.0
            
            # 为每个电机设置不同的目标位置
            for motor_id in MOTOR_IDS_TO_TEST:
                target_pos = math.radians(0.0)
                driver.set_mit_mode(motor_id, target_pos, vel_target, kp, kd, torque)
                print(f"  > 已发送MIT指令给电机 {hex(motor_id)}: 目标 {math.degrees(target_pos):.1f}°")


            # # ======================================================================
            # # --- 示例2: 位置速度模式 ---
            # # 目标: 让两个电机以1 rad/s的速度运动到相反的180度位置。
            # # ======================================================================
            # print("\n测试: 位置速度模式")
            # target_vel = 1.0
            # for motor_id in MOTOR_IDS_TO_TEST:
            #     target_pos = math.radians(180.0) if motor_id == MOTOR_IDS_TO_TEST[0] else math.radians(-180.0)
            #     driver.set_pos_vel_mode(motor_id, target_pos, target_vel)
            #     print(f"  > 已发送位置速度指令给电机 {hex(motor_id)}: 目标 {math.degrees(target_pos):.1f}°")
            

            # # ======================================================================
            # # --- 示例3: 速度模式 ---
            # # 目标: 让两个电机以相反的速度持续转动。
            # # ======================================================================
            # print("\n测试: 速度模式")
            # for motor_id in MOTOR_IDS_TO_TEST:
            #     target_vel = 2.0 if motor_id == MOTOR_IDS_TO_TEST[0] else -2.0
            #     driver.set_vel_mode(motor_id, target_vel)
            #     print(f"  > 已发送速度指令给电机 {hex(motor_id)}: 速度 {target_vel} rad/s")
            

            # # ======================================================================
            # # --- 示例4: 力位混合模式 ---
            # # 目标: 让两个电机运动到相反的45度位置，同时施加一个力矩。
            # # ======================================================================
            # print("\n测试: 力位混合模式")
            # vel_des = 100  # SDK特定单位
            # i_des = 1000   # SDK特定单位
            # for motor_id in MOTOR_IDS_TO_TEST:
            #     target_pos = math.radians(45.0) if motor_id == MOTOR_IDS_TO_TEST[0] else math.radians(-45.0)
            #     driver.set_torque_pos_mode(motor_id, target_pos, vel_des, i_des)
            #     print(f"  > 已发送力位混合指令给电机 {hex(motor_id)}: 目标 {math.degrees(target_pos):.1f}°")


            # --- 通用反馈循环 ---
            print("\n--- 开始监控反馈 (按 Ctrl+C 停止) ---")
            while True:
                feedback_str = ""
                # 遍历每个电机，并独立获取其最新的反馈
                for motor_id in MOTOR_IDS_TO_TEST:
                    feedback = driver.get_feedback(motor_id) # 这一步现在完成了所有工作
                    if feedback:
                        pos_deg = math.degrees(feedback['position'])
                        vel_rads = feedback['velocity']
                        tor_nm = feedback['torque']
                        feedback_str += f"ID {hex(motor_id)} -> Pos:{pos_deg:7.2f}° Vel:{vel_rads:6.2f} Tor:{tor_nm:5.2f} Nm | "
                    else:
                        feedback_str += f"ID {hex(motor_id)} -> [获取失败] | "
                
                print(f"\r{feedback_str}", end="", flush=True) # 使用动态刷新
                time.sleep(0.05) # 调整循环频率

    except KeyboardInterrupt:
        print("\n\n程序被用户中断。")
    except Exception as e:
        print(f"\n程序主流程发生严重错误: {e}")
        print("请检查硬件连接和配置。")

    print("\n示例程序执行完毕。")
