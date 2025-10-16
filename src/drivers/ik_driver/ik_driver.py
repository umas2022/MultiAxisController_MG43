import numpy as np
import math
import time
import csv

class CombinedMotionController:
    JOINT_NAMES = [
        'LF_HAA', 'LF_HFE', 'LF_KFE',
        'RF_HAA', 'RF_HFE', 'RF_KFE',
        'LH_HAA', 'LH_HFE', 'LH_KFE',
        'RH_HAA', 'RH_HFE', 'RH_KFE'
    ]

    def __init__(self):
        # 大腿长度/m
        self.l_thigh = 0.13
        # 小腿长度/m
        self.l_shank = 0.17
        # 身体高度/m
        self.z_body = -0.26
        # 直行单步长度/m
        self.step_length_linear = 0.15
        # 转向单步长度/m
        self.step_length_angular = 0.15
        self.lift_height = 0.05
        # 步态周期/s
        self.period = 2
        # 控制频率/Hz
        self.Hz = 50
        self.dt = 1.0 / self.Hz

        self.init_angles = [0.0] * 12
        self.phases = {'LF': 0.0, 'RF': math.pi, 'LH': math.pi, 'RH': 0.0}
        self.leg_order = ['LF', 'RF', 'LH', 'RH']

        # 关节符号
        self.joint_signs = {
            'LF': {'HAA': +1.0, 'HFE': +1.0, 'KFE': -1.0},
            'RF': {'HAA': -1.0, 'HFE': +1.0, 'KFE': -1.0},
            'LH': {'HAA': +1.0, 'HFE': +1.0, 'KFE': -1.0},
            'RH': {'HAA': -1.0, 'HFE': +1.0, 'KFE': -1.0},
        }

        self.leg_xy_signs = {
            'LF': np.array([+1.0, +1.0]), 'RF': np.array([+1.0, -1.0]),
            'LH': np.array([-1.0, +1.0]), 'RH': np.array([-1.0, -1.0]),
        }

        self.t = 0.0
        
        # --- 新增状态变量 ---
        # is_moving 用于判断机器人是否应该运动
        self.is_moving = False
        # 缓存站立时的关节角度，避免重复计算
        self._standing_angles = self._compute_standing_angles()

    @staticmethod
    def ik_leg_2d(x, z, thigh, shank):
        # 防止因计算误差或目标点超出范围导致 acos 的参数越界
        D_arg = (x**2 + z**2 - thigh**2 - shank**2) / (2 * thigh * shank)
        D_arg = np.clip(D_arg, -1.0, 1.0)
        # 注意 atan2(y, x) 中 y 是 sin, x 是 cos。sqrt(1-D^2) 总是正的，对应于“膝盖向前弯曲”的解。
        theta2_raw = math.atan2(math.sqrt(1 - D_arg**2), -D_arg) 
        # atan2(x, -z) 是腿部连线与Z轴的夹角。第二个 atan2 是余弦定理的修正项
        theta1 = math.atan2(x, -z) - math.atan2(shank * math.sin(theta2_raw), thigh - shank * math.cos(theta2_raw))
        return theta1, theta2_raw

    @staticmethod
    def ik_leg_3d_haa_xaxis(x, y, z, thigh, shank):
        # 从侧面看，计算HAA关节角度
        haa = math.atan2(y, -z)
        # 计算旋转到XZ平面后，足端到HAA关节的距离
        z_prime = -math.sqrt(y**2 + z**2) if z < 0 else math.sqrt(y**2 + z**2)
        # 在XZ平面上进行2D IK求解
        hfe, kfe_raw = CombinedMotionController.ik_leg_2d(x, z_prime, thigh, shank)
        return haa, hfe, kfe_raw

    @staticmethod
    def leg_foot_trajectory(phase, lift_height):
        # 支撑相 (Stance phase): 脚在地面上，从前半程移动到后半程
        if phase < math.pi:
            # s_progress 从 -0.5 线性变化到 0.5
            s_progress = -0.5 + (phase / math.pi)
            dz = 0.0
        # 摆动相 (Swing phase): 脚在空中抬起，从后半程移动到前半程
        else:
            # s_progress 从 0.5 线性变化到 -0.5
            s_progress = 0.5 - ((phase - math.pi) / math.pi)
            # dz 是一个半正弦波，实现平滑的抬腿和落腿
            dz = lift_height * math.sin(phase - math.pi)
        return s_progress, dz

    def apply_joint_signs(self, leg, haa, hfe, kfe):
        s = self.joint_signs[leg]
        return s['HAA'] * haa, s['HFE'] * hfe, s['KFE'] * kfe

    def _compute_standing_angles(self):
        """计算并返回静止站立时的关节角度"""
        standing_angles = []
        for leg in self.leg_order:
            # 站立时，所有腿的目标点都在身体正下方
            # (x=0, y=0, z=z_body)
            haa_geo, hfe_geo, kfe_raw_geo = self.ik_leg_3d_haa_xaxis(
                0.0, 0.0, self.z_body, self.l_thigh, self.l_shank
            )
            kfe_geo = math.pi - kfe_raw_geo
            haa_cmd, hfe_cmd, kfe_cmd = self.apply_joint_signs(leg, haa_geo, hfe_geo, kfe_geo)
            standing_angles.extend([haa_cmd, hfe_cmd, kfe_cmd])
        
        final_angles = [a + ia for a, ia in zip(standing_angles, self.init_angles)]
        return final_angles

    def get_motor_angles(self, linear_x, linear_y, angular_z):
        # 1. 判断速度指令是否为零
        eps = 0.01  # 容许小范围的噪声
        is_command_zero = (abs(linear_x) < eps and abs(linear_y) < eps and abs(angular_z) < eps)

        # 2. 如果指令为零，直接返回站立角度
        if is_command_zero:
            return self._standing_angles

        # 3. 如果指令不为零，则标记为正在移动
        if not is_command_zero:
            self.is_moving = True

        # --- 以下为步态计算逻辑 (移动或正在停止时执行) ---
        
        linear_dir = np.array([linear_x, linear_y])
        linear_norm = np.linalg.norm(linear_dir)
        if linear_norm > 1.0:
            linear_dir /= linear_norm
        
        angular_z = np.clip(angular_z, -1.0, 1.0)

        motion_angles = []

        for leg in self.leg_order:
            phase = (2.0 * math.pi * (self.t / self.period) + self.phases[leg]) % (2.0 * math.pi)
            s_progress, dz = self.leg_foot_trajectory(phase, self.lift_height)
            
            # 如果指令为零(正在停止的过程中)，步长也应为零，让脚落回原位
            current_step_length_linear = 0 if is_command_zero else self.step_length_linear
            current_step_length_angular = 0 if is_command_zero else self.step_length_angular

            # 平移位移
            foot_disp_linear = s_progress * current_step_length_linear * (-linear_dir)
            
            # 旋转位移
            yaw_dir = 1.0 if angular_z >= 0 else -1.0
            xy_sign = self.leg_xy_signs[leg]
            tang_base = np.array([-xy_sign[1], xy_sign[0]])
            tang = yaw_dir * (tang_base / np.linalg.norm(tang_base))
            foot_disp_angular = s_progress * current_step_length_angular * abs(angular_z) * tang
            
            # 叠加
            final_foot_disp = foot_disp_linear + foot_disp_angular
            
            x_body = float(final_foot_disp[0])
            y_body = float(final_foot_disp[1])
            z_body = self.z_body + dz

            # IK和后续处理
            # 身体坐标系下的x对于前后腿IK计算时的符号是相反的
            # 这里保持了原有的逻辑，假设它是正确的
            if leg in ["LF", "RF"]:
                x_for_ik = x_body
            else:
                x_for_ik = -x_body

            haa_geo, hfe_geo, kfe_raw_geo = self.ik_leg_3d_haa_xaxis(
                x_for_ik, y_body, z_body, self.l_thigh, self.l_shank
            )
            kfe_geo = math.pi - kfe_raw_geo
            haa_cmd, hfe_cmd, kfe_cmd = self.apply_joint_signs(leg, haa_geo, hfe_geo, kfe_geo)
            
            motion_angles.extend([haa_cmd, hfe_cmd, kfe_cmd])

        final_angles = [a + ia for a, ia in zip(motion_angles, self.init_angles)]
        return final_angles

    def update(self, linear_x, linear_y, angular_z):
        """根据指令更新内部时钟和状态"""
        is_command_zero = (linear_x == 0 and linear_y == 0 and angular_z == 0)

        # 如果指令为零且已经静止，则不更新时间
        if is_command_zero and not self.is_moving:
            return

        # 如果在移动或正在过渡到停止，则推进时间
        self.t += self.dt

        # 如果正在从移动过渡到停止
        if self.is_moving and is_command_zero:
            # 检查是否到达半周期边界，这是Trot步态的稳定停止点
            # 使用取模运算判断
            time_in_half_period = self.t % (self.period / 2.0)
            
            # 如果在一个dt内将要跨过边界点，就停止
            # (self.t + dt) % half_period < time_in_half_period 是一个更鲁棒的边界检测方法
            if time_in_half_period < self.dt:
                self.is_moving = False
                # 重置时间，防止浮点数误差累积，并为下次启动做准备
                self.t = 0.0

