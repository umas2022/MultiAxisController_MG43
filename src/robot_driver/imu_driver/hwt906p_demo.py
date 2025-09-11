# coding:UTF-8
import time
import platform
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver


def onUpdate(imu_device:deviceModel.DeviceModel):
    """ 数据更新时回调，打印核心数据 """
    print(
        "加速度: ({:.3f}, {:.3f}, {:.3f}) g".format(
            imu_device.getDeviceData("accX"),
            imu_device.getDeviceData("accY"),
            imu_device.getDeviceData("accZ"),
        ),
        "角速度: ({:.3f}, {:.3f}, {:.3f}) °/s".format(
            imu_device.getDeviceData("gyroX"),
            imu_device.getDeviceData("gyroY"),
            imu_device.getDeviceData("gyroZ"),
        ),
        "角度: ({:.3f}, {:.3f}, {:.3f}) °".format(
            imu_device.getDeviceData("angleX"),
            imu_device.getDeviceData("angleY"),
            imu_device.getDeviceData("angleZ"),
        ),
    )


if __name__ == '__main__':

    # 初始化设备
    device = deviceModel.DeviceModel(
        "HWT906P",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
    )

    # 设置串口
    device.serialConfig.portName = "COM3"
    device.serialConfig.baud = 921600

    # 打开设备
    device.openDevice()

    # 注册回调函数
    device.dataProcessor.onVarChanged.append(onUpdate)

    print("开始打印传感器数据，按 Ctrl+C 停止。")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n已退出。")
    finally:
        device.closeDevice()
