#!/usr/bin/env python3
import serial
import threading
import time
import sys

# --- CRSF 协议常量 ---
CRSF_ADDRESS = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
EXPECTED_PAYLOAD_LENGTH = 22


def parse_crsf_channels(payload):
    """从22字节的payload中解析16个通道数据"""
    if len(payload) != EXPECTED_PAYLOAD_LENGTH:
        return None

    bits = int.from_bytes(payload, byteorder="little")
    channels = []
    for i in range(16):
        value = (bits >> (i * 11)) & 0x7FF  # 11位
        channels.append(value)
    return channels


class ElrsReceiver:
    def __init__(self, serial_port="/dev/ttyUSB0", baud_rate=420000):
        self.serial_port = serial_port
        self.baud_rate = baud_rate

        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            
        self.buffer = bytearray()
        self.latest_channels = [992] * 16
        self.running = True

        # 通道取值范围
        # 通道0：右摇杆y轴
        self.CHANNEL_0_MIN = 174
        self.CHANNEL_0_MAX = 1811
        # 通道1：右摇杆x轴
        self.CHANNEL_1_MIN = 174
        self.CHANNEL_1_MAX = 1811
        # 通道3：左摇杆yaw轴
        self.CHANNEL_3_MIN = 174   
        self.CHANNEL_3_MAX = 1811

        # 开启串口读取线程
        self.serial_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.serial_thread.start()
        time.sleep(0.5)

    def _serial_read_loop(self):
        """串口读取和解析的循环"""
        # try:
        #     self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
        #     self.running = True
        # except Exception as e:
        #     print("elrs serial error: ",e)
        #     sys.exit(1)

        while self.running:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    self.buffer.extend(data)

                    # 提取合法帧
                    while len(self.buffer) >= 3:
                        if self.buffer[0] != CRSF_ADDRESS:
                            self.buffer.pop(0)
                            continue

                        frame_len = self.buffer[1]
                        if not (2 < frame_len < 64):
                            self.buffer.pop(0)
                            continue

                        total_len = frame_len + 2
                        if len(self.buffer) < total_len:
                            break

                        packet = self.buffer[:total_len]
                        self.buffer = self.buffer[total_len:]

                        if packet[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED and frame_len == EXPECTED_PAYLOAD_LENGTH + 2:
                            payload = packet[3:-1]
                            channels = parse_crsf_channels(payload)
                            if channels:
                                self.latest_channels = channels

            except serial.SerialException as e:
                print(f"[ELRS] Serial port error: {e}")
                self.running = False
                break
            except Exception as e:
                print(f"[ELRS] Unexpected error: {e}")
            time.sleep(0.005)

    def get_all_channel_raw(self):
        """获取当前16通道值 (列表形式, 每个值 174~1811)"""
        if self.running:
            return self.latest_channels.copy()
        else:
            raise RuntimeError("elrs resolver not running")
            

    def get_channel_raw(self, index):
        """获取单个通道值 (index: 0~15)"""
        if 0 <= index < 16:
            return self.latest_channels[index]
        return None
    
    def get_channel_xyz(self)-> tuple:
        """
        获取x,y,z轴归一化的值
        return: (x, y, z) 范围[-1, 1]
        """
        x_raw = self.get_channel_raw(1)  # 右摇杆x轴
        y_raw = self.get_channel_raw(0)  # 右摇杆y轴
        z_raw = self.get_channel_raw(3)  # 左摇杆yaw轴

        x_norm = (x_raw - self.CHANNEL_1_MIN) / (self.CHANNEL_1_MAX - self.CHANNEL_1_MIN) * 2 - 1
        y_norm = (y_raw - self.CHANNEL_0_MIN) / (self.CHANNEL_0_MAX - self.CHANNEL_0_MIN) * 2 - 1
        z_norm = (z_raw - self.CHANNEL_3_MIN) / (self.CHANNEL_3_MAX - self.CHANNEL_3_MIN) * 2 - 1

        x_norm = max(-1.0, min(1.0, x_norm))
        y_norm = max(-1.0, min(1.0, y_norm))
        z_norm = max(-1.0, min(1.0, z_norm))

        return x_norm, y_norm, z_norm

    def close(self):
        """关闭串口和线程"""
        self.running = False
        if self.serial_thread.is_alive():
            self.serial_thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()


# --- 示例用法 ---
if __name__ == "__main__":
    elrs = ElrsReceiver("/dev/ttyUSB0", 420000)
    if not elrs.running:
        print("elrs not running, exit ...")
        sys.exit(1)
    try:
        while True:
            chans = elrs.get_all_channel_raw()
            print("ELRS Channels:", chans)
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        elrs.close()
