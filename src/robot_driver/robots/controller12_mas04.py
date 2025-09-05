import math
import time
from typing import List, Dict, Optional, Tuple
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from src.robot_driver.motor_driver.dm_can_driver import DMMotorDriver, DM_Motor_Type


@dataclass
class JointConfig:
    id: int
    reversed: bool = False  # 是否反转方向
    offset: float = 0.0  # 零点微调（单位: rad）


class Controller12:
    # 关节配置表：每个关节一个配置
    JOINT_CONFIGS = [
        JointConfig(0x01, reversed=False, offset=math.radians(0.0)),  # LF_HAA
        JointConfig(0x02, reversed=True, offset=math.radians(0.0)),  # LF_HFE
        JointConfig(0x03, reversed=False, offset=math.radians(0.0)),  # LF_KFE

        JointConfig(0x04, reversed=True, offset=math.radians(0.0)),  # RF_HAA
        JointConfig(0x05, reversed=False, offset=math.radians(0.0)),  # RF_HFE
        JointConfig(0x06, reversed=True, offset=math.radians(0.0)),  # RF_KFE

        JointConfig(0x07, reversed=True, offset=math.radians(0.0)),  # LH_HAA
        JointConfig(0x08, reversed=False, offset=math.radians(0.0)),    # LH_HFE
        JointConfig(0x09, reversed=True, offset=math.radians(0.0)),    # LH_KFE

        JointConfig(0x0A, reversed=False, offset=math.radians(0.0)),  # RH_HAA
        JointConfig(0x0B, reversed=True, offset=math.radians(0.0)),   # RH_HFE
        JointConfig(0x0C, reversed=False, offset=math.radians(0.0)),   # RH_KFE
    ]

    # 默认的MIT模式控制参数 (可以为不同关节设置不同值)
    DEFAULT_KP = 20.0
    DEFAULT_KD = 0.2

    def __init__(self, port: str, motor_type: DM_Motor_Type = DM_Motor_Type.DM4310):
        """
        初始化四足机器人。

        :param port: 连接电机驱动器的串口号。
        :param motor_type: 所有关节电机的型号。
        """
        print("--- 初始化四足机器人 ---")
        self.driver = DMMotorDriver(port=port)

        print("正在注册12个关节电机...")
        for joint_obj in self.JOINT_CONFIGS:
            self.driver.add_motor(joint_obj.id, motor_type)

        if not self.check_online():
            raise RuntimeError("初始化失败：部分关节未响应。请检查连接。")

        print("正在使能所有关节...")
        self.driver.enable_all()
        time.sleep(1)  # 等待电机使能稳定
        print("--- 四足机器人初始化完成，所有关节已使能 ---")

    def check_online(self) -> bool:
        """
        检查所有12个关节是否在线。
        """
        print("\n检查所有关节是否在线...")
        retrys = 3
        all_online = True
        for joint_obj in self.JOINT_CONFIGS:
            feedback = self.driver.get_feedback(joint_obj.id)
            initial_pos = feedback["position"] if feedback else None
            initial_vel = feedback["velocity"] if feedback else None
            initial_tau = feedback["torque"] if feedback else None
            if initial_pos and initial_vel and initial_tau:
                print(
                    f"关节 ID {hex(joint_obj.id)} 在线，位置: {math.degrees(initial_pos):.2f}°"
                )
            else:
                while retrys > 0:
                    time.sleep(0.1)
                    feedback = self.driver.get_feedback(joint_obj.id)
                    retry_pos = feedback["position"] if feedback else None
                    retry_vel = feedback["velocity"] if feedback else None
                    retry_tau = feedback["torque"] if feedback else None
                    if retry_pos and retry_vel and retry_tau:
                        print(
                            f"关节 ID {hex(joint_obj.id)} 在线，位置: {math.degrees(retry_pos):.2f}°"
                        )
                        break
                    retrys -= 1
                if retrys == 0:
                    print(f"错误：关节 ID {hex(joint_obj.id)} 未响应！请检查连接。")
                    all_online = False
        return all_online

    def home_joints_pv_mode(self, velocity: float = 0.5, tol_deg: float = 4.0):
        """
        以pos_vel模式让所有12个关节回到逻辑零点位置（考虑反转和零点偏移）。
        :param velocity: 回零速度 (rad/s)
        :param tol_deg: 允许误差 (度)
        """
        print(f"\n执行归零动作...")

        # 目标 = 逻辑零点 (即用户视角的0)
        target_positions = [0.0] * len(self.JOINT_CONFIGS)
        self.switch_all_mode_pos_vel()
        self.move_all_pos_vel_mode(target_positions, [velocity]*12)

        tol_rad = math.radians(tol_deg)

        while True:
            all_reached = True
            for cfg in self.JOINT_CONFIGS:
                feedback = self.driver.get_feedback(cfg.id)
                if feedback:
                    pos = feedback["position"]

                    # 转换到逻辑坐标系：去掉offset并恢复reversed
                    logical_pos = (-1 if cfg.reversed else 1) * (pos - cfg.offset)

                    # 判断是否在零点附近
                    if abs(logical_pos) > tol_rad:
                        all_reached = False
                        break

                    # 堵转安全检测
                    torque = feedback["torque"]
                    if abs(torque) > 3:
                        self.shutdown()
                        raise RuntimeError(
                            f"错误：关节 ID {hex(cfg.id)} 可能堵转，已紧急停止机器人！"
                        )
                else:
                    all_reached = False
                    break

            if all_reached:
                print("所有关节已到达零点位置。")
                break
            time.sleep(0.1)

    def home_joints_mit_mode(
        self,
        velocity: float = 0.5,
        tol_deg: float = 4.0,
        interpolation_steps: int = 100,
        kp: List[float] = None,
        kd: List[float] = None,
    ):
        """
        以 MIT 模式让所有12个关节通过平滑插值回到逻辑零点位置。

        :param velocity: 插值速度 (rad/s)。
        :param tol_deg: 允许误差 (度)。
        :param interpolation_steps: 插值步数，步数越多，运动越平滑。
        :param kp: Kp 系数列表，默认为 None，将使用 DEFAULT_KP。
        :param kd: Kd 系数列表，默认为 None，将使用 DEFAULT_KD。
        """
        print("\n执行 MIT 模式归零动作...")

        if kp is None:
            kp = [self.DEFAULT_KP] * len(self.JOINT_CONFIGS)
        if kd is None:
            kd = [self.DEFAULT_KD] * len(self.JOINT_CONFIGS)

        # 1. 切换模式
        self.switch_all_mode_mit()
        time.sleep(0.1)  # 留出时间让电机完成模式切换

        # 2. 获取当前位置作为插值起点
        start_positions = []
        for cfg in self.JOINT_CONFIGS:
            feedback = self.driver.get_feedback(cfg.id)
            if not feedback:
                self.shutdown()
                raise RuntimeError("无法获取电机反馈，已紧急停止。")
            
            # 将物理位置转换为逻辑位置
            pos = feedback["position"]
            logical_pos = (-1 if cfg.reversed else 1) * (pos - cfg.offset)
            start_positions.append(logical_pos)
        
        # 3. 定义目标位置（逻辑零点）
        target_positions = [0.0] * len(self.JOINT_CONFIGS)
        
        # 4. 插值并发送指令
        # 步长 = (目标位置 - 起始位置) / 总步数
        step_positions = [(target_positions[i] - start_positions[i]) / interpolation_steps 
                          for i in range(len(self.JOINT_CONFIGS))]
        
        # 计算每个插值步的延时，确保运动速度大致恒定
        # time_per_step = (最长距离 / velocity) / interpolation_steps
        max_dist = max(abs(diff) for diff in [target_positions[i] - start_positions[i] for i in range(len(self.JOINT_CONFIGS))])
        if max_dist == 0:
            print("所有关节已在零点位置。")
            return
        
        total_time = max_dist / velocity
        time_per_step = total_time / interpolation_steps
        
        for i in range(interpolation_steps + 1):
            current_target = [start_positions[j] + step_positions[j] * i 
                              for j in range(len(self.JOINT_CONFIGS))]
            
            self.move_all_mit_mode(
                pos=current_target,
                kp=kp,
                kd=kd,
            )
            time.sleep(time_per_step)

        # 5. 最终检查和堵转安全检测
        tol_rad = math.radians(tol_deg)
        while True:
            all_reached = True
            for cfg in self.JOINT_CONFIGS:
                feedback = self.driver.get_feedback(cfg.id)
                if not feedback:
                    all_reached = False
                    break
                
                # 转换到逻辑坐标系：去掉offset并恢复reversed
                pos = feedback["position"]
                logical_pos = (-1 if cfg.reversed else 1) * (pos - cfg.offset)
                
                if abs(logical_pos) > tol_rad:
                    all_reached = False
                    break

                # 堵转安全检测
                torque = feedback["torque"]
                if abs(torque) > 3:  # 假设堵转扭矩阈值为3
                    self.shutdown()
                    raise RuntimeError(
                        f"错误：关节 ID {hex(cfg.id)} 可能堵转，已紧急停止机器人！"
                    )

            if all_reached:
                print("所有关节已到达零点位置。")
                break
            time.sleep(0.1)

    def get_all_joint_feedback(self) -> Dict[int, Optional[Dict[str, float]]]:
        """
        获取所有12个关节的最新反馈数据。

        :return: 一个字典，键为关节ID，值为该关节的反馈数据字典。
        """
        all_feedback = {}
        for joint_obj in self.JOINT_CONFIGS:
            feedback = self.driver.get_feedback(joint_obj.id)
            all_feedback[joint_obj.id] = feedback
        return all_feedback 


    def switch_all_mode_mit(self):
        """
        切换所有关节到 MIT 模式
        切换模式时电机会短暂失能，避免在线切换
        """
        for cfg in self.JOINT_CONFIGS:
            self.driver.switch_mode_mit(cfg.id)

    def switch_all_mode_pos_vel(self):
        """
        切换所有关节到 位置-速度 模式
        切换模式时电机会短暂失能，避免在线切换
        """
        for cfg in self.JOINT_CONFIGS:
            self.driver.switch_mode_pos_vel(cfg.id)

    def move_all_mit_mode(
        self,
        pos: List[float],
        vel: List[float] = [0.0]*12,
        kp: List[float] = [DEFAULT_KP]*12,
        kd: List[float] = [DEFAULT_KD]*12,
        torque: List[float] = [0.0]*12,
    ):
        """
        以 MIT 模式控制所有关节
        首先调用 switch_all_mode_mit() 切换模式
        """
        n = len(self.JOINT_CONFIGS)
        if not all(len(lst) == n for lst in [pos, vel, kp, kd, torque]):
            raise ValueError("所有输入参数的长度必须等于12个关节")

        # 使用简单的for循环串行执行
        for i, (cfg, p, v, kpi, kdi, t) in enumerate(zip(self.JOINT_CONFIGS, pos, vel, kp, kd, torque)):
            # 方向与零点修正
            real_pos = (-p if cfg.reversed else p) + cfg.offset
            real_vel = -v if cfg.reversed else v
            real_torque = -t if cfg.reversed else t

            # 直接调用驱动函数，串行执行
            self.driver.move_mit_mode(
                cfg.id, real_pos, real_vel, kpi, kdi, real_torque
            )

    def move_all_pos_vel_mode(self, target_positions, velocity=[1.0] * 12):
        """
        以位置-速度模式控制所有关节
        首先调用 switch_all_mode_pos_vel() 切换模式
        """
        if len(target_positions) != len(self.JOINT_CONFIGS):
            raise ValueError("目标位置数量必须等于12个关节")

        # 使用简单的for循环串行执行
        for cfg, target, vel in zip(self.JOINT_CONFIGS, target_positions, velocity):
            # 方向与零点修正
            real_target = (-target if cfg.reversed else target) + cfg.offset
            
            # 直接调用驱动函数，串行执行
            self.driver.set_pos_vel_mode(cfg.id, real_target, vel)


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
    SERIAL_PORT = "COM15"  # 修改为你的实际串口

    robot = None
    try:
        # 创建机器人实例，初始化和使能电机
        robot = Controller12(port=SERIAL_PORT)

        # robot.home_joints_pv_mode()
        # time.sleep(2)
        # print("\n--- 进入站立姿态 (位置-速度模式) ---")
        # robot.switch_all_mode_pos_vel()
        # robot.move_all_pos_vel_mode(
        #     target_positions=[
        #         0, -0.4, -0.8,  # LF
        #         0, -0.4, -0.8,  # RF
        #         0, -0.4, -0.8,  # LH
        #         0, -0.4, -0.8,  # RH
        #     ],
        #     velocity=[1]*12
        # )

        robot.home_joints_mit_mode()
        time.sleep(2)
        print("\n--- 进入站立姿态 (MIT模式) ---")
        robot.switch_all_mode_mit()
        robot.move_all_mit_mode(
            pos=[
                0, -0.4, -0.8,  # LF
                0, -0.4, -0.8,  # RF
                0, -0.4, -0.8,  # LH
                0, -0.4, -0.8,  # RH
            ]
        )

        # 持续监控机器人状态
        print("\n--- 开始监控所有关节状态 (按 Ctrl+C 停止) ---")
        while True:
            # 获取所有关节的反馈
            joint_states = robot.get_all_joint_feedback()

            # 格式化打印
            print("=" * 80)
            for i in range(0, 12, 3):
                row_str = ""
                for j in range(3):
                    joint_id = robot.JOINT_CONFIGS[i + j].id
                    state = joint_states.get(joint_id)
                    if state:
                        pos_deg = math.degrees(state["position"])
                        row_str += f"ID {hex(joint_id):<4s}: {pos_deg:7.2f}° | "
                    else:
                        row_str += f"ID {hex(joint_id):<4s}: [No Data] | "
                print(row_str)

            time.sleep(1)  # 每秒刷新一次

    except KeyboardInterrupt:
        print("\n程序被用户中断。")
    except Exception as e:
        print(f"\n程序发生错误: {e}")
    finally:
        # 无论发生什么，都确保机器人被安全关闭
        if robot:
            robot.shutdown()
