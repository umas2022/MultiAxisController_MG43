import math
import time
from typing import List, Dict, Optional, Tuple
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from src.drivers.motor_driver.motor_driver import  MotorDriver, DM_Motor_Type


@dataclass
class JointConfig:
    id: int
    reversed: bool = False  # 是否反转方向
    offset: float = 0.0  # 零点微调（单位: rad）


class RobotDriverMas04:

    JOINT_CONFIGS:List[JointConfig] = [
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

    # In Isaac Sim, the joint ordering is as follows:
    # [
    #     'LF_HAA', 'LH_HAA', 'RF_HAA', 'RH_HAA',
    #     'LF_HFE', 'LH_HFE', 'RF_HFE', 'RH_HFE',
    #     'LF_KFE', 'LH_KFE', 'RF_KFE', 'RH_KFE'
    # ]
    ISAAC_ORDER = [0,6,3,9,1,7,4,10,2,8,5,11]


    # 默认的MIT模式控制参数 (可以为不同关节设置不同值)
    DEFAULT_KP = 30.0
    DEFAULT_KD = 0.5

    # 默认初始位置
    DEFAULT_POS = [0.0, -0.4, -0.8] * 4  # [LF, RF, LH, RH]
    DEFAULT_POS_ISAAC = [0.0]*4 + [-0.4]*4 + [-0.8]*4  # Isaac Sim的顺序

    def __init__(self, port: str, motor_type: DM_Motor_Type = DM_Motor_Type.DM4310):
        """
        初始化四足机器人。

        :param port: 连接电机驱动器的串口号。
        :param motor_type: 所有关节电机的型号。
        """
        print("--- 初始化四足机器人 ---")
        self.driver = MotorDriver(port=port)

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
        velocity: float = 0.3,
        tol_deg: float = 4.0,
        min_step_deg: float = 1.0,
        kp: List[float] = None,
        kd: List[float] = None,
    ):
        """
        以 MIT 模式让所有关节回到逻辑零点位置。
        :param velocity: 速度 (rad/s)。
        :param tol_deg: 允许误差 (度)。
        :param min_step_deg: 每个插值步的最小角度 (度)。
        """
        print("\n执行 MIT 模式归零动作...")

        target_positions = [0.0] * len(self.JOINT_CONFIGS)

        self.move_all_mit_mode_smooth(
            target_positions=target_positions,
            velocity=velocity,
            min_step_deg=min_step_deg,
            kp=kp,
            kd=kd,
        )

        # 4. 检查是否到位（附带堵转检测）
        tol_rad = math.radians(tol_deg)
        while True:
            all_reached = True
            for cfg in self.JOINT_CONFIGS:
                feedback = self.driver.get_feedback(cfg.id)
                if not feedback:
                    all_reached = False
                    break

                pos = feedback["position"]
                logical_pos = (-1 if cfg.reversed else 1) * (pos - cfg.offset)

                if abs(logical_pos) > tol_rad:
                    all_reached = False
                    break

                # 堵转检测
                torque = feedback["torque"]
                if abs(torque) > 3:
                    self.shutdown()
                    raise RuntimeError(
                        f"错误：关节 ID {hex(cfg.id)} 可能堵转，已紧急停止机器人！"
                    )

            if all_reached:
                print("所有关节已到达零点位置。")
                break
            time.sleep(0.1)


    def get_logical_pos(self, joint_id: int,raw_pos: float) -> float:
        '''
        将原始位置转换为逻辑位置。
        '''
        cfg = next((c for c in self.JOINT_CONFIGS if c.id == joint_id), None)
        if not cfg:
            raise ValueError(f"未找到关节 ID {hex(joint_id)} 的配置。")
        logical_pos = (-1 if cfg.reversed else 1) * (raw_pos - cfg.offset)
        return logical_pos
    
    def get_logical_vel(self, joint_id: int,raw_vel: float) -> float:
        '''
        将原始速度转换为逻辑速度。
        '''
        cfg = next((c for c in self.JOINT_CONFIGS if c.id == joint_id), None)
        if not cfg:
            raise ValueError(f"未找到关节 ID {hex(joint_id)} 的配置。")
        logical_vel = -raw_vel if cfg.reversed else raw_vel
        return logical_vel
    
    def get_logical_tau(self, joint_id: int,raw_tau: float) -> float:
        '''
        将原始力矩转换为逻辑力矩。
        '''
        cfg = next((c for c in self.JOINT_CONFIGS if c.id == joint_id), None)
        if not cfg:
            raise ValueError(f"未找到关节 ID {hex(joint_id)} 的配置。")
        logical_tau = -raw_tau if cfg.reversed else raw_tau
        return logical_tau


    def get_all_feedback(self) -> Dict[int, Optional[Dict[str, float]]]:
        """
        获取所有12个关节的最新反馈数据，程序运行时间0.001~0.003s。

        :return:
        {
            0x01: {"position": 0.0, "velocity": 0.0, "torque": 0.0},
            0x02: {"position": 0.1, "velocity": 0.0, "torque": 0.1},
            ...
            0x0C: None
        """
        all_feedback = {}
        for joint_obj in self.JOINT_CONFIGS:
            feedback = self.driver.get_feedback(joint_obj.id)
            if feedback:
                feedback["position"] = self.get_logical_pos(joint_obj.id, feedback["position"])
                feedback["velocity"] = self.get_logical_vel(joint_obj.id, feedback["velocity"])
                feedback["torque"] = self.get_logical_tau(joint_obj.id, feedback["torque"])
            all_feedback[joint_obj.id] = feedback
        return all_feedback 

    def get_all_feedback_pvt(self) -> Tuple[List[float], List[float], List[float]]:
        '''
        以[pos, vel, tau]的形式获取所有12个关节的最新反馈数据，程序运行时间0.001~0.003s。
        '''
        positions = []
        velocities = []
        torques = []
        for joint_obj in self.JOINT_CONFIGS:
            feedback = self.driver.get_feedback(joint_obj.id)
            if feedback:
                positions.append(self.get_logical_pos(joint_obj.id, feedback["position"]))
                velocities.append(self.get_logical_vel(joint_obj.id, feedback["velocity"]))
                torques.append(self.get_logical_tau(joint_obj.id, feedback["torque"]))
            else:
                positions.append(None)
                velocities.append(None)
                torques.append(None)
        return positions, velocities, torques   
    
    def get_all_feedback_isaac(self) -> Tuple[List[float], List[float], List[float]]:
        '''
        以[pos, vel, tau]的形式获取所有12个关节的最新反馈数据，pos返回相对于DEFAULT_POS的角度，程序运行时间0.001~0.003s。
        '''
        positions, velocities, torques = self.get_all_feedback_pvt()
        positions = [positions[i] - self.DEFAULT_POS[i] if positions[i] is not None else None for i in range(12)]
        # 重新排序以匹配Isaac Sim的顺序
        positions = [positions[i] for i in self.ISAAC_ORDER]
        velocities = [velocities[i] for i in self.ISAAC_ORDER]
        torques = [torques[i] for i in self.ISAAC_ORDER]
        return positions, velocities, torques


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
        需要首先调用 switch_all_mode_mit() 切换模式
        程序运行时间约0.0015~0.0027s
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

    def move_all_mit_mode_smooth(
        self,
        target_positions: List[float],
        velocity: float = 0.5,
        min_step_deg: float = 1.0,
        kp: List[float] = None,
        kd: List[float] = None,
    ):
        """
        MIT模式通用平滑运动函数，支持插值拆分（基于固定周期调度）。
        注意此模式微步速度精度较差，建议仅用于位置初始化
        """

        if kp is None:
            kp = [self.DEFAULT_KP] * len(self.JOINT_CONFIGS)
        if kd is None:
            kd = [self.DEFAULT_KD] * len(self.JOINT_CONFIGS)

        # 1. 获取当前位置作为起点
        start_positions = []
        for cfg in self.JOINT_CONFIGS:
            feedback = self.driver.get_feedback(cfg.id)
            if not feedback:
                self.shutdown()
                raise RuntimeError("无法获取电机反馈，已紧急停止。")

            pos = feedback["position"]
            logical_pos = (-1 if cfg.reversed else 1) * (pos - cfg.offset)
            start_positions.append(logical_pos)

        # 2. 计算最大运动距离
        diffs = [target_positions[i] - start_positions[i] for i in range(len(self.JOINT_CONFIGS))]
        max_dist = max(abs(d) for d in diffs)
        if max_dist == 0:
            print("所有关节已在目标位置。")
            return

        # 3. 计算总时间、插值步数
        total_time = max_dist / velocity
        min_step_rad = math.radians(min_step_deg)
        interpolation_steps = int(max(max_dist / min_step_rad, 1))
        step_positions = [diffs[i] / interpolation_steps for i in range(len(self.JOINT_CONFIGS))]
        time_per_step = float(total_time / interpolation_steps)

        # 4. 插值执行（基于固定周期调度）
        start_time = time.perf_counter()
        for i in range(interpolation_steps + 1):
            current_target = [start_positions[j] + step_positions[j] * i
                              for j in range(len(self.JOINT_CONFIGS))]
            self.move_all_mit_mode(
                pos=current_target,
                kp=kp,
                kd=kd,
            )

            # 计算下一个目标时间点
            next_time = start_time + (i + 1) * time_per_step
            now = time.perf_counter()
            sleep_time = next_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)



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
            self.driver.move_pos_vel_mode(cfg.id, real_target, vel)


    def shutdown(self):
        """
        安全地关闭机器人，失能所有电机并关闭串口。
        """
        print("\n--- 开始关闭机器人 ---")
        self.driver.close()
        print("--- 机器人已安全关闭 ---")


