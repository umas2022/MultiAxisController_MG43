
'''
示例: 使用训练好的策略模型进行推理
'''
import numpy as np
import sys
from pathlib import Path

# 添加项目根目录到Python路径
sys.path.insert(0, str(Path(__file__).parent.parent))
from src.models.policy_runner_pt import PolicyRunnerPT


# ===========================
# 使用示例
# ===========================
if __name__ == "__main__":
    # --- 用户配置 ---
    MODEL_PATH = "..\src\models\pt\model_2025-09-11_17-12-00.pt"
    NUM_OBS = 3 + 3 + 3 + 3 + 12 + 12 + 12
    DEFAULT_JOINT_POS = np.array([0.0, -0.4, -0.8] * 4, dtype=np.float32)

    # --- 创建策略封装器 ---
    policy = PolicyRunnerPT(
        model_path=MODEL_PATH,
        num_actions=12,
        num_obs=NUM_OBS,
        default_joint_pos=DEFAULT_JOINT_POS,
        action_scale=0.25,
    )

    # --- 构造观测向量（示例: 全零） ---
    obs_vector = np.zeros(NUM_OBS, dtype=np.float32)

    # --- 获取电机目标角 ---
    target_pos = policy.infer_action(obs_vector)

    print("\n推理结果:")
    print("目标关节角度 (rad):")
    print(target_pos)
