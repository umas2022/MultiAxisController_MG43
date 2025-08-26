import math
import time
from typing import List, Dict, Optional

# 从你的项目结构中导入DMMotorDriver
from src.robot_driver.motor_driver.dm_can_driver import DMMotorDriver, DM_Motor_Type

class QuadrupedRobot:
    """
    一个四足机器人控制类，用于管理12个关节电机，并提供高级动作接口。
    """
    
    # 机器人关节的物理和控制配置
    # 建议将这些ID与你的URDF或实际硬件的命名对应起来
    JOINT_IDS = [
        # 前左腿 (Front-Left)
        0x01,  # LF_HAA (Hip Abduction/Adduction)
        0x02,  # LF_HFE (Hip Flextion/Extension)
        0x03,  # LF_KFE (Knee Flextion/Extension)
        # 前右腿 (Front-Right)
        0x04,  # RF_HAA
        0x05,  # RF_HFE
        0x06,  # RF_KFE
        # 后左腿 (Hind-Left)
        0x07,  # LH_HAA
        0x08,  # LH_HFE
        0x09,  # LH_KFE
        # 后右腿 (Hind-Right)
        0x0A,  # RH_HAA
        0x0B,  # RH_HFE
        0x0C,  # RH_KFE
    ]
    
    # 默认的MIT模式控制参数 (可以为不同关节设置不同值)
    DEFAULT_KP = 15.0
    DEFAULT_KD = 0.8

    def __init__(self, port: str, motor_type: DM_Motor_Type = DM_Motor_Type.DM4310):
        """
        初始化四足机器人。
        
        :param port: 连接电机驱动器的串口号。
        :param motor_type: 所有关节电机的型号。
        """
        print("--- 初始化四足机器人 ---")
        self.driver = DMMotorDriver(port=port)
        
        print("正在注册12个关节电机...")
        for joint_id in self.JOINT_IDS:
            self.driver.add_motor(joint_id, motor_type)
        
        print("正在使能所有关节...")
        self.driver.enable_all()
        time.sleep(1) # 等待电机使能稳定
        print("--- 四足机器人初始化完成，所有关节已使能 ---")

    def home_all_joints(self, mode: str = "MIT", speed: float = 0.0, torque: float = 0.0):
        """
        让所有12个关节回到零点位置。
        
        :param mode: 控制模式，可选 "MIT", "POS_VEL", "VEL"。默认为 "MIT"。
        :param speed: 在 "POS_VEL" 或 "VEL" 模式下的目标速度 (rad/s)。
        :param torque: 在 "MIT" 模式下的前馈力矩 (Nm)。
        """
        print(f"\n执行归零动作，使用模式: {mode}")
        
        target_pos_rad = 0.0
        
        for joint_id in self.JOINT_IDS:
            if mode == "MIT":
                self.driver.set_mit_mode(
                    motor_id=joint_id, 
                    pos_rad=target_pos_rad, 
                    vel_rad_s=0.0,  # 归零时目标速度为0
                    kp=self.DEFAULT_KP, 
                    kd=self.DEFAULT_KD, 
                    torque_nm=torque
                )
            elif mode == "POS_VEL":
                self.driver.set_pos_vel_mode(
                    motor_id=joint_id,
                    pos_rad=target_pos_rad,
                    vel_rad_s=speed
                )
            elif mode == "VEL":
                # 速度模式下没有“位置”概念，最接近“归零”的是让它停止转动
                self.driver.set_vel_mode(
                    motor_id=joint_id,
                    vel_rad_s=0.0
                )
            elif mode == "TORQUE_POS":
                # 这种模式较复杂，这里仅作演示
                print("警告: TORQUE_POS模式需要特定的vel_des和i_des参数，此处仅演示归零。")
                self.driver.set_torque_pos_mode(
                    motor_id=joint_id,
                    pos_rad=target_pos_rad,
                    vel_des=0, # SDK特定单位
                    i_des=0    # SDK特定单位
                )
            else:
                print(f"错误: 不支持的控制模式 '{mode}'")
                return
        
        print("所有关节的归零指令已发送。")

    def get_all_joint_feedback(self) -> Dict[int, Optional[Dict[str, float]]]:
        """
        获取所有12个关节的最新反馈数据。
        
        :return: 一个字典，键为关节ID，值为该关节的反馈数据字典。
        """
        all_feedback = {}
        for joint_id in self.JOINT_IDS:
            # 内部已包含请求-响应流程
            feedback = self.driver.get_feedback(joint_id)
            all_feedback[joint_id] = feedback
        return all_feedback

    def shutdown(self):
        """
        安全地关闭机器人，失能所有电机并关闭串口。
        """
        print("\n--- 开始关闭机器人 ---")
        self.driver.close()
        print("--- 机器人已安全关闭 ---")


# ======================================================================
#                            使用示例
# ======================================================================
if __name__ == "__main__":
    # --- 配置 ---
    SERIAL_PORT = 'COM15'  # 修改为你的实际串口

    robot = None # 初始化变量，确保finally块中可用
    try:
        # 1. 创建机器人实例，这会自动初始化和使能电机
        robot = QuadrupedRobot(port=SERIAL_PORT)

        # 2. 发送高级动作指令
        # 默认使用MIT模式让所有电机回到0位
        robot.home_all_joints()

        # # --- 如果想切换模式，可以这样做 (取消注释来测试) ---
        # print("\n--- 切换到 POS_VEL 模式测试 ---")
        # time.sleep(2) # 等待上个动作完成
        # robot.home_all_joints(mode="POS_VEL", speed=1.0) # 让电机以1.0 rad/s的速度回到0点

        # 3. 持续监控机器人状态
        print("\n--- 开始监控所有关节状态 (按 Ctrl+C 停止) ---")
        while True:
            # 获取所有关节的反馈
            joint_states = robot.get_all_joint_feedback()
            
            # 格式化打印
            print("="*80)
            for i in range(0, 12, 3):
                row_str = ""
                for j in range(3):
                    joint_id = robot.JOINT_IDS[i+j]
                    state = joint_states.get(joint_id)
                    if state:
                        pos_deg = math.degrees(state['position'])
                        row_str += f"ID {hex(joint_id):<4s}: {pos_deg:7.2f}° | "
                    else:
                        row_str += f"ID {hex(joint_id):<4s}: [No Data] | "
                print(row_str)
            
            time.sleep(1) # 每秒刷新一次

    except KeyboardInterrupt:
        print("\n程序被用户中断。")
    except Exception as e:
        print(f"\n程序主流程发生严重错误: {e}")
    finally:
        # 无论发生什么，都确保机器人被安全关闭
        if robot:
            robot.shutdown()
