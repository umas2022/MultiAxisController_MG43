'''
示例：elrs接收机
左摇杆yaw轴：通道3
右摇杆x轴：通道1
右摇杆y轴：通道0
'''
import sys
import time
from pathlib import Path

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.drivers.elrs_driver.elrs_driver import ElrsReceiver

elrs = ElrsReceiver("COM3", 420000)
try:
    while True:
        # chans = elrs.get_all_channel_raw()
        # print("ELRS Channels:", chans)

        x, y, z = elrs.get_channel_xyz()
        print(f"ELRS x:{x:.3f} y:{y:.3f} z:{z:.3f}")

        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    elrs.close()