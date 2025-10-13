'''
逆运动学动作求解(不使用rl模型)
'''

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
        self.step_length_linear = 0.1
        # 转向单步长度/m
        self.step_length_angular = 0.1
        self.lift_height = 0.05
        # 步态周期/s
        self.period = 0.25
        self.Hz = 100
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

    @staticmethod
    def ik_leg_2d(x, z, thigh, shank):
        D = (x**2 + z**2 - thigh**2 - shank**2) / (2 * thigh * shank)
        D = np.clip(D, -1.0, 1.0)
        theta2_raw = math.atan2(math.sqrt(1 - D**2), -D)
        theta1 = math.atan2(x, -z) - math.atan2(shank * math.sin(theta2_raw), thigh - shank * math.cos(theta2_raw))
        return theta1, theta2_raw

    @staticmethod
    def ik_leg_3d_haa_xaxis(x, y, z, thigh, shank):
        haa = math.atan2(y, -z)
        z_prime = -math.sqrt(y**2 + z**2) if z < 0 else math.sqrt(y**2 + z**2)
        hfe, kfe_raw = CombinedMotionController.ik_leg_2d(x, z_prime, thigh, shank)
        return haa, hfe, kfe_raw

    @staticmethod
    def leg_foot_trajectory(phase, lift_height):
        if phase < math.pi:
            s_progress = -0.5 + (phase / math.pi)
            dz = 0.0
        else:
            s_progress = 0.5 - ((phase - math.pi) / math.pi)
            dz = lift_height * math.sin(phase - math.pi)
        return s_progress, dz

    def apply_joint_signs(self, leg, haa, hfe, kfe):
        s = self.joint_signs[leg]
        return s['HAA'] * haa, s['HFE'] * hfe, s['KFE'] * kfe

    def get_motor_angles(self, linear_x, linear_y, angular_z):
        linear_dir = np.array([linear_x, linear_y])
        linear_norm = np.linalg.norm(linear_dir)
        if linear_norm > 1.0:
            linear_dir /= linear_norm
        
        angular_z = np.clip(angular_z, -1.0, 1.0)

        motion_angles = []

        for leg in self.leg_order:
            phase = (2.0 * math.pi * (self.t / self.period) + self.phases[leg]) % (2.0 * math.pi)
            s_progress, dz = self.leg_foot_trajectory(phase, self.lift_height)
            
            # 平移位移
            foot_disp_linear = s_progress * self.step_length_linear * (-linear_dir)
            
            # 旋转位移
            yaw_dir = 1.0 if angular_z >= 0 else -1.0
            xy_sign = self.leg_xy_signs[leg]
            tang_base = np.array([-xy_sign[1], xy_sign[0]])
            tang = yaw_dir * (tang_base / np.linalg.norm(tang_base))
            foot_disp_angular = s_progress * self.step_length_angular * abs(angular_z) * tang
            
            # 叠加
            final_foot_disp = foot_disp_linear + foot_disp_angular
            
            x_body = float(final_foot_disp[0])
            y_body = float(final_foot_disp[1])
            z_body = self.z_body + dz

            # IK和后续处理
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

    def update(self):
        self.t += self.dt

# # 使用示例：
# if __name__ == '__main__':
#     controller = CombinedMotionController()
#     while True:
#         # 输入目标方向
#         linear_x = 1.0
#         linear_y = 0.0
#         angular_z = 0.0
        
#         angles = controller.get_motor_angles(linear_x, linear_y, angular_z)
#         print(f"电机角度: {angles}")

#         # 更新时间
#         controller.update()
#         time.sleep(controller.dt)


# 使用示例：
if __name__ == '__main__':
    controller = CombinedMotionController()

    # 打开 CSV 文件进行写入
    with open('motor_angles.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        # 写入CSV头部
        writer.writerow(['Time', 'LF_HAA', 'LF_HFE', 'LF_KFE', 'RF_HAA', 'RF_HFE', 'RF_KFE', 'LH_HAA', 'LH_HFE', 'LH_KFE', 'RH_HAA', 'RH_HFE', 'RH_KFE'])
        
        # 循环记录角度
        start_time = time.time()
        counter = 0
        while True:
            # 输入目标方向
            linear_x = 1.0
            linear_y = 0.0
            angular_z = 0.0
            
            angles = controller.get_motor_angles(linear_x, linear_y, angular_z)
            current_time = time.time() - start_time  # 计算经过的时间
            
            # 写入当前时间和电机角度到 CSV 文件
            writer.writerow([current_time] + angles)

            # 更新时间
            controller.update()
            time.sleep(controller.dt)

            counter += 1
            if counter >= 500:  # 记录500次后退出
                break
            print(counter)