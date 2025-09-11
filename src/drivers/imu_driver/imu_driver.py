# src/drivers/imu_driver/imu_driver.py
# coding:UTF-8
import time
import threading
from typing import Tuple, Dict

from src.drivers.imu_driver.lib.device_model import DeviceModel
from src.drivers.imu_driver.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from src.drivers.imu_driver.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver


class IMUDriver:
    """机器人 IMU 驱动封装，支持获取加速度、角速度、角度（9轴数据）"""

    def __init__(self, port: str = "COM3", baud: int = 921600, dev_name: str = "HWT906P"):
        self.device = DeviceModel(
            dev_name,
            WitProtocolResolver(),
            JY901SDataProcessor(),
            "51_0"   # 设备编号，可以保持默认
        )
        self.device.serialConfig.portName = port
        self.device.serialConfig.baud = baud

        # 保存最近一次数据
        self._latest_data: Dict[str, float] = {
            "accX": 0.0, "accY": 0.0, "accZ": 0.0,
            "gyroX": 0.0, "gyroY": 0.0, "gyroZ": 0.0,
            "angleX": 0.0, "angleY": 0.0, "angleZ": 0.0,
        }
        self._lock = threading.Lock()

        # 注册回调
        self.device.dataProcessor.onVarChanged.append(self._on_update)

    def _on_update(self, imu_device: DeviceModel):
        """内部回调，更新最新数据"""
        with self._lock:
            for key in self._latest_data.keys():
                self._latest_data[key] = imu_device.getDeviceData(key)

    def start(self):
        """打开设备"""
        self.device.openDevice()

    def stop(self):
        """关闭设备"""
        self.device.closeDevice()

    def get_data(self) -> Tuple[Tuple[float, float, float],
                                Tuple[float, float, float],
                                Tuple[float, float, float]]:
        """获取最新的 (加速度, 角速度, 角度)"""
        with self._lock:
            acc = (self._latest_data["accX"], self._latest_data["accY"], self._latest_data["accZ"])
            gyro = (self._latest_data["gyroX"], self._latest_data["gyroY"], self._latest_data["gyroZ"])
            angle = (self._latest_data["angleX"], self._latest_data["angleY"], self._latest_data["angleZ"])
        return acc, gyro, angle
