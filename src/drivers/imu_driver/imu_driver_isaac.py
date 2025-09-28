# src/drivers/imu_driver/imu_driver.py
# coding:UTF-8
import time
import threading
from typing import Tuple, Dict

import numpy as np
from scipy.spatial.transform import Rotation

from src.drivers.imu_driver.lib.device_model import DeviceModel
from src.drivers.imu_driver.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from src.drivers.imu_driver.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver


class IMUDriver:
    """
    机器人 IMU 驱动封装。
    包含一个【实验性】的速度积分估算功能，用于评估漂移。
    """

    def __init__(self, port: str = "COM3", baud: int = 921600, dev_name: str = "HWT906P", g: float = 9.81):
        self.device = DeviceModel(
            dev_name,
            WitProtocolResolver(),
            JY901SDataProcessor(),
            "51_0"
        )
        self.device.serialConfig.portName = port
        self.device.serialConfig.baud = baud
        
        self.G = g  # 重力加速度

        # 保存最近一次原始数据
        self._latest_data: Dict[str, float] = {
            "accX": 0.0, "accY": 0.0, "accZ": 0.0,
            "gyroX": 0.0, "gyroY": 0.0, "gyroZ": 0.0,
            "angleX": 0.0, "angleY": 0.0, "angleZ": 0.0,
        }
        
        # --- 用于速度积分的状态变量 ---
        # 在世界坐标系下的速度估计 (vx, vy, vz)
        self._velocity_world = np.array([0.0, 0.0, 0.0])
        self._last_update_time = None # 上次更新时间戳
        self._lock = threading.Lock()

        # 注册回调
        self.device.dataProcessor.onVarChanged.append(self._on_update)

    # def _on_update(self, imu_device: DeviceModel):
    #     """
    #     内部回调，更新最新数据并执行速度积分。
    #     """
    #     current_time = time.perf_counter()

    #     with self._lock:
    #         # 1. 更新原始传感器数据
    #         for key in self._latest_data.keys():
    #             self._latest_data[key] = imu_device.getDeviceData(key)
            
    #         # 如果是第一次更新，只记录时间，不进行积分
    #         if self._last_update_time is None:
    #             self._last_update_time = current_time
    #             return

    #         dt = current_time - self._last_update_time
    #         if dt <= 0: # 避免时间间隔无效
    #             return

    #         # --- 速度积分计算 ---
    #         # a. 获取当前姿态和加速度
    #         acc_body = np.array([self._latest_data["accX"], self._latest_data["accY"], self._latest_data["accZ"]]) * self.G
    #         angle_deg = np.array([self._latest_data["angleX"], self._latest_data["angleY"], self._latest_data["angleZ"]])
            
    #         # b. 创建旋转对象，用于坐标系转换
    #         rotation = Rotation.from_euler('xyz', angle_deg, degrees=True)
            
    #         # c. 从身体坐标系的加速度中移除重力分量，得到纯运动加速度
    #         # 首先计算世界重力在身体坐标系下的投影
    #         gravity_in_body = rotation.inv().apply(np.array([0, 0, -self.G]))
    #         # 从总加速度中减去重力投影，得到运动加速度（身体坐标系）
    #         motion_acc_body = acc_body + gravity_in_body
            
    #         # d. 将运动加速度转换到世界坐标系下
    #         # 这是为了在固定的坐标系下进行积分，避免旋转带来的复杂性
    #         motion_acc_world = rotation.apply(motion_acc_body)
            
    #         # e. 积分更新世界坐标系下的速度 (v = v0 + a*dt)
    #         self._velocity_world += motion_acc_world * dt

    #         # f. 更新时间戳
    #         self._last_update_time = current_time


    def _on_update(self, imu_device: DeviceModel):
        """
        内部回调，更新最新数据并执行速度积分。
        增加了静止检测 + 自动归零（ZUPT），用于在静止时抑制漂移。
        """
        current_time = time.perf_counter()

        with self._lock:
            # 1. 更新原始传感器数据
            for key in self._latest_data.keys():
                self._latest_data[key] = imu_device.getDeviceData(key)

            # 如果是第一次更新，只记录时间，不进行积分
            if self._last_update_time is None:
                self._last_update_time = current_time
                return

            dt = current_time - self._last_update_time
            if dt <= 0:  # 避免时间间隔无效
                return

            # --- 速度积分计算 ---
            # a. 获取当前姿态和加速度
            acc_body = np.array([
                self._latest_data["accX"],
                self._latest_data["accY"],
                self._latest_data["accZ"],
            ]) * self.G
            angle_deg = np.array([
                self._latest_data["angleX"],
                self._latest_data["angleY"],
                self._latest_data["angleZ"],
            ])

            # b. 创建旋转对象，用于坐标系转换
            rotation = Rotation.from_euler("xyz", angle_deg, degrees=True)

            # c. 从身体坐标系的加速度中移除重力分量，得到运动加速度
            gravity_in_body = rotation.inv().apply(np.array([0, 0, -self.G]))
            motion_acc_body = acc_body + gravity_in_body

            # d. 将运动加速度转换到世界坐标系
            motion_acc_world = rotation.apply(motion_acc_body)

            # e. 积分更新速度
            self._velocity_world += motion_acc_world * dt

            # f. 静止检测 (ZUPT)
            #   - 如果加速度接近重力，且角速度接近 0，就认为静止
            acc_norm = np.linalg.norm(acc_body)
            gyro_norm = np.linalg.norm([
                self._latest_data["gyroX"],
                self._latest_data["gyroY"],
                self._latest_data["gyroZ"],
            ])

            if abs(acc_norm - self.G) < 0.1 * self.G and gyro_norm < 1.0:  
                # 阈值可以根据实际传感器调节
                self._velocity_world[:] = 0.0

            # g. 更新时间戳
            self._last_update_time = current_time


    def start(self):
        """打开设备"""
        self.device.openDevice()

    def stop(self):
        """关闭设备"""
        self.device.closeDevice()

    def reset_velocity_integration(self):
        """重置速度积分状态，用于重新开始测试。"""
        with self._lock:
            self._velocity_world = np.array([0.0, 0.0, 0.0])
            self._last_update_time = None
            print("Velocity integration has been reset.")

    def get_isaaclab_data(self) -> Tuple[Tuple[float, float, float],
                                         Tuple[float, float, float],
                                         Tuple[float, float, float]]:
        """
        获取 Isaac Lab 训练所需的9轴数据。
        - 线速度通过【积分】得到（实验性）。
        - 角速度和重力投影直接从传感器获取。
        """
        with self._lock:
            # 1. 获取角速度 (rad/s)
            gyro_dps = (self._latest_data["gyroX"], self._latest_data["gyroY"], self._latest_data["gyroZ"])
            base_angular_velocity_rad = tuple(np.deg2rad(val) for val in gyro_dps)

            # 2. 获取姿态，计算重力投影（以g为单位，而不是m/s²）
            angle_deg = (self._latest_data["angleX"], self._latest_data["angleY"], self._latest_data["angleZ"])
            rotation = Rotation.from_euler('xyz', angle_deg, degrees=True)
            gravity_vector_world = np.array([0, 0, -1.0])  # 单位向量，1g而不是9.8m/s²
            projected_gravity_body = tuple(rotation.inv().apply(gravity_vector_world))

            # 3. 获取积分得到的速度
            # 当前速度是在世界坐标系下的 (_velocity_world)，需要转换回机器人身体坐标系
            velocity_body = tuple(rotation.inv().apply(self._velocity_world))
            base_linear_velocity = velocity_body

        return base_linear_velocity, base_angular_velocity_rad, projected_gravity_body


