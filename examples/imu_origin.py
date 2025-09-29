
'''
示例：打印IMU原始9轴数据
'''
import sys
from pathlib import Path

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.drivers.imu_driver.imu_driver_origin import IMUDriver
import time

imu = IMUDriver(port="COM3", baud=921600)
imu.start()

try:
    while True:
        acc, gyro, angle = imu.get_data()
        print(f"加速度: {acc}, 角速度: {gyro}, 角度: {angle}")
        time.sleep(0.1)
except KeyboardInterrupt:
    imu.stop()
