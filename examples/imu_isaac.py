
import sys
from pathlib import Path

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.drivers.imu_driver.imu_driver_isaac import IMUDriver
import time

# --- 使用示例 ---
if __name__ == '__main__':
    imu = IMUDriver(port="COM3", baud=921600)
    imu.start()
    print("IMU starting... Please keep the robot stationary for a few seconds.")
    time.sleep(3) # 等待设备稳定

    try:
        print("\n--- Testing Drift ---")
        print("Robot should be perfectly still. Observing velocity drift over time...")
        for i in range(200): # 观察10秒
            lin_vel, ang_vel, grav = imu.get_isaaclab_data()
            
            # print(f"Time: {i*0.5:.1f}s | Est. Velocity (m/s): "
            #       f"({lin_vel[0]:.4f}, {lin_vel[1]:.4f}, {lin_vel[2]:.4f})")
            
            print(f"Time: {i*0.5:.1f}s | Est. Angular Velocity (rad/s): "
                  f"({ang_vel[0]:.4f}, {ang_vel[1]:.4f}, {ang_vel[2]:.4f})")
            
            # print(f"Time: {i*0.5:.1f}s | Est. Gravity (g): "
            #       f"({grav[0]:.4f}, {grav[1]:.4f}, {grav[2]:.4f})")
            
            # 你会看到，即使机器人不动，这个速度值也会逐渐增大（漂移）
            
            time.sleep(0.5)

    finally:
        imu.stop()
        print("\nIMU stopped.")