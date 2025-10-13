'''
将csv中预先录制的obs输入到模型，输出保存到csv中
（replay_csv/replay_csv.py可以用输出的csv驱动机器人）
'''
import torch
import numpy as np
import pandas as pd

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
# 主程序
# ================================

if __name__ == '__main__':
    # --- 用户配置 ---
    MODEL_PATH = r"..\..\src\models\pt\model_2025-09-11_17-12-00.pt"  
    # INPUT_CSV_PATH = r"obs_stand_2025-09-11_17-12-00.csv"                  
    INPUT_CSV_PATH = r"obs_go_2025-09-11_17-12-00.csv"                    
    OUTPUT_CSV_PATH = r"action.csv"              

    NUM_ACTIONS = 12
    NUM_HEIGHT_SCAN_POINTS = 0
    NUM_OBS = 3 + 3 + 3 + 3 + 12 + 12 + 12 + NUM_HEIGHT_SCAN_POINTS
    DEFAULT_JOINT_POS = np.array([0.0] * 4+[-0.4]*4+[-0.8]*4, dtype=np.float32)
    ACTION_SCALE = 0.25

    # --- 加载策略 ---
    policy_net = load_rsl_rl_policy(MODEL_PATH, NUM_ACTIONS, NUM_OBS)

    # --- 读取输入 CSV ---
    try:
        df_input = pd.read_csv(INPUT_CSV_PATH)
        print(f"成功读取输入文件: '{INPUT_CSV_PATH}'")
    except FileNotFoundError:
        print(f"错误: 找不到文件 '{INPUT_CSV_PATH}'。请检查路径。")
        exit()

    # 准备输出 DataFrame
    output_columns = [f"pos_{i}" for i in range(NUM_ACTIONS)]
    df_output = pd.DataFrame(columns=['time'] + output_columns)

    # --- 逐行处理数据并进行推理 ---
    for index, row in df_input.iterrows():
        # 根据您提供的 CSV 头部，提取观测数据
        obs_parts = [
            row[['base_lin_vel_0', 'base_lin_vel_1', 'base_lin_vel_2']].values,
            row[['base_ang_vel_0', 'base_ang_vel_1', 'base_ang_vel_2']].values,
            row[['projected_gravity_0', 'projected_gravity_1', 'projected_gravity_2']].values,
            row[['velocity_commands_0', 'velocity_commands_1', 'velocity_commands_2']].values,
            row[[f"joint_pos_{i}" for i in range(12)]].values,
            row[[f"joint_vel_{i}" for i in range(12)]].values,
            row[[f"actions_{i}" for i in range(12)]].values,
        ]

        # 线速度覆盖为目标速度
        obs_parts[0] = row[['velocity_commands_0', 'velocity_commands_1', 'velocity_commands_2']].values.astype(np.float32)
        # # 角速度覆盖为零
        # obs_parts[1] = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        # # 重力向量覆盖为 [0, 0, -1]
        # obs_parts[2] = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        # # joint_pos覆盖为默认站立位置
        # obs_parts[4] = DEFAULT_JOINT_POS
        # # joint_vel覆盖为零
        # obs_parts[5] = np.array([0.0] * 12, dtype=np.float32)

        obs_vector = np.concatenate(obs_parts, axis=0).astype(np.float32)

        if len(obs_vector) != NUM_OBS:
            print(f"警告: 第 {index} 行观测维度不匹配。跳过。")
            continue

        # --- 推理 ---
        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs_vector).unsqueeze(0)  # 添加 batch 维度
            raw_model_output = policy_net.act_inference(obs_tensor)
            model_output_np = raw_model_output.squeeze(0).cpu().numpy()

        # --- 计算最终目标关节角 ---
        final_target_joint_pos = DEFAULT_JOINT_POS + ACTION_SCALE * model_output_np

        # --- 将结果添加到输出 DataFrame ---
        output_row = [row['time']] + list(final_target_joint_pos)
        df_output.loc[index] = output_row

    # --- 保存结果到新的 CSV 文件 ---
    df_output.to_csv(OUTPUT_CSV_PATH, index=False)
    print(f"\n模型输出已成功保存到 '{OUTPUT_CSV_PATH}'。")