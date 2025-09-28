
'''
全零输入pt模型play测试脚本
'''


#!/usr/bin/env python3
import torch
import numpy as np

# 我们只需要 ActorCritic，因为归一化器在它内部
from rsl_rl.modules import ActorCritic


def load_rsl_rl_policy(model_path, num_actions, num_obs):
    """
    加载一个由 RSL-RL 框架训练的、内含归一化器的策略模型。
    """
    policy = ActorCritic(
        num_actor_obs=num_obs,
        num_critic_obs=num_obs,
        num_actions=num_actions,
        actor_hidden_dims=[128, 128, 128],
        critic_hidden_dims=[128, 128, 128],
        activation='elu',
    )

    loaded_dict = torch.load(model_path, map_location='cpu', weights_only=True)
    policy.load_state_dict(loaded_dict['model_state_dict'])
    policy.eval()

    print(f"模型已从 '{model_path}' 加载成功。")
    return policy


# ================================
# 各类观测的模拟/读取函数 (可替换为真实传感器)
# ================================

def get_base_linear_velocity() -> np.ndarray:
    """获取基座线速度 [vx, vy, vz]"""
    # TODO: 替换为来自IMU或状态估计的真实值
    return np.array([0.0, 0.0, 0.0], dtype=np.float32)


def get_base_angular_velocity() -> np.ndarray:
    """获取基座角速度 [wx, wy, wz]"""
    # TODO: 来自 IMU 的角速度
    return np.array([0.0, 0.0, 0.0], dtype=np.float32)


def get_projected_gravity() -> np.ndarray:
    """获取重力在机体坐标系下的投影 [gx, gy, gz]"""
    # TODO: 来自 IMU 或姿态估计算法
    return np.array([0.0, 0.0, -1.0], dtype=np.float32)


def get_velocity_commands() -> np.ndarray:
    """获取当前的速度指令 [v_cmd_x, v_cmd_y, v_cmd_yaw]"""
    # TODO: 来自上层控制器或遥控指令
    return np.array([0.0, 0.0, 0.0], dtype=np.float32)


def get_joint_positions() -> np.ndarray:
    """获取12个关节的当前位置（弧度）"""
    # TODO: 来自电机编码器反馈
    return np.zeros(12, dtype=np.float32)


def get_joint_velocities() -> np.ndarray:
    """获取12个关节的当前速度（弧度/秒）"""
    # TODO: 来自电机编码器微分或直接测量
    return np.zeros(12, dtype=np.float32)


def get_previous_actions() -> np.ndarray:
    """获取上一时刻的网络输出动作（用于记忆或滤波）"""
    # TODO: 来自上一控制周期的动作缓存
    return np.zeros(12, dtype=np.float32)


# 可选：高度扫描（如果后续使用）
def get_height_scan() -> np.ndarray:
    """获取地形高度扫描点（若无则返回空数组）"""
    return np.array([], dtype=np.float32)


# ================================
# 主程序
# ================================

if __name__ == '__main__':
    # --- 用户配置 ---
    MODEL_PATH = "..\..\src\models\pt\model_2025-09-11_17-12-00.pt"
    NUM_ACTIONS = 12
    NUM_HEIGHT_SCAN_POINTS = 0  # 当前未使用
    NUM_OBS = 3 + 3 + 3 + 3 + 12 + 12 + 12 + NUM_HEIGHT_SCAN_POINTS
    DEFAULT_JOINT_POS = np.array([0.0, -0.4, -0.8] * 4, dtype=np.float32)
    ACTION_SCALE = 0.25

    # --- 加载策略 ---
    policy_net = load_rsl_rl_policy(MODEL_PATH, NUM_ACTIONS, NUM_OBS)

    # --- 构建观测向量 ---
    obs_parts = [
        get_base_linear_velocity(),       # 3D
        get_base_angular_velocity(),      # 3D
        get_projected_gravity(),          # 3D
        get_velocity_commands(),          # 3D
        get_joint_positions(),            # 12D
        get_joint_velocities(),           # 12D
        get_previous_actions(),           # 12D
        get_height_scan()                 # N-D (0 here)
    ]

    obs_vector = np.concatenate(obs_parts, axis=0).astype(np.float32)

    assert len(obs_vector) == NUM_OBS, f"观测维度不匹配: 期望 {NUM_OBS}, 实际 {len(obs_vector)}"

    print(f"\n创建的模拟输入 (前12维): {obs_vector[:12]}")

    # --- 推理 ---
    with torch.no_grad():
        obs_tensor = torch.from_numpy(obs_vector).unsqueeze(0)  # 添加 batch 维度
        raw_model_output = policy_net.act_inference(obs_tensor)
        model_output_np = raw_model_output.squeeze(0).cpu().numpy()

    # --- 输出目标关节角 ---
    final_target_joint_pos = DEFAULT_JOINT_POS + ACTION_SCALE * model_output_np

    # --- 打印结果 ---
    print("\n--------------------------------------------------------")
    print("原始模型输出 (Action):")
    print(model_output_np)
    print("\n最终计算出的电机目标角度 (rad):")
    print(final_target_joint_pos)
    print("--------------------------------------------------------")
