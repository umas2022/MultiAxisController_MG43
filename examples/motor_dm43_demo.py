
'''
示例：【MIT模式/速度位置模式】驱动DM43电机，并持续打印反馈
'''
import sys
from pathlib import Path

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.drivers.motor_driver.motor_driver import MotorDriver, DM_Motor_Type
import math
import time


# --- 配置 ---
PORT = 'COM15'  # 修改为你的串口
MOTOR_IDS_TO_TEST = [0x03] 

try:
    # 使用 'with' 语句自动管理资源的打开和关闭，非常安全
    with MotorDriver(port=PORT) as driver:
        # 1. 添加并使能所有需要的电机
        for motor_id in MOTOR_IDS_TO_TEST:
            driver.add_motor(motor_id, DM_Motor_Type.DM4310)
        
        driver.enable_all()
        print(f"\n已使能电机: {[hex(mid) for mid in MOTOR_IDS_TO_TEST]}")
        time.sleep(1) # 等待电机使能完成

        print("\n--- 开始测试电机控制模式 ---")

        # # ======================================================================
        # # --- 示例1: MIT模式  ---
        # # ======================================================================
        # print("\n测试: MIT模式")
        # driver.switch_all_mode_mit() # 切换所有电机到MIT模式

        # kp, kd, torque = 5.0, 0.5, 0.0
        # vel_target = 0.0
        
        # # 为每个电机设置不同的目标位置
        # for motor_id in MOTOR_IDS_TO_TEST:
        #     target_pos = math.radians(0.0)
        #     driver.move_mit_mode(motor_id, target_pos, vel_target, kp, kd, torque)
        #     print(f"  > 已发送MIT指令给电机 {hex(motor_id)}: 目标 {math.degrees(target_pos):.1f}°")


        # ======================================================================
        # --- 示例2: 位置速度模式 ---
        # ======================================================================
        print("\n测试: 位置速度模式")
        driver.switch_all_mode_pos_vel() # 切换所有电机到位置速度模式

        target_vel = 1.0
        for motor_id in MOTOR_IDS_TO_TEST:
            target_pos = math.radians(0.0)
            driver.move_pos_vel_mode(motor_id, target_pos, target_vel)
            print(f"  > 已发送位置速度指令给电机 {hex(motor_id)}: 目标 {math.degrees(target_pos):.1f}°")


        # --- 通用反馈循环 ---
        print("\n--- 开始监控反馈 (按 Ctrl+C 停止) ---")
        while True:
            feedback_str = ""
            # 遍历每个电机，并独立获取其最新的反馈
            for motor_id in MOTOR_IDS_TO_TEST:
                feedback = driver.get_feedback(motor_id) # 这一步现在完成了所有工作
                if feedback:
                    pos_deg = math.degrees(feedback['position'])
                    vel_rads = feedback['velocity']
                    tor_nm = feedback['torque']
                    feedback_str += f"ID {hex(motor_id)} -> Pos:{pos_deg:7.2f}° Vel:{vel_rads:6.2f} Tor:{tor_nm:5.2f} Nm | "
                else:
                    feedback_str += f"ID {hex(motor_id)} -> [获取失败] | "
            
            print(f"\r{feedback_str}", end="", flush=True) # 使用动态刷新
            time.sleep(0.05) # 调整循环频率

except KeyboardInterrupt:
    print("\n\n程序被用户中断。")
except Exception as e:
    print(f"\n程序主流程发生严重错误: {e}")
    print("请检查硬件连接和配置。")

print("\n示例程序执行完毕。")
