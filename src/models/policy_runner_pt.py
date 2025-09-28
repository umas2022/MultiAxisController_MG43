'''
pip install rsl-rl-lib
'''

import torch
import numpy as np
from rsl_rl.modules import ActorCritic


class PolicyRunnerPT:
    """
    RSL-RL 策略模型封装类
    - 自动加载 ActorCritic
    - 内置归一化
    - 输入观测向量，输出目标关节角
    """

    def __init__(self, model_path: str, num_actions: int, num_obs: int,
                 default_joint_pos: np.ndarray, action_scale: float = 0.25):
        self.model_path = model_path
        self.num_actions = num_actions
        self.num_obs = num_obs
        self.default_joint_pos = default_joint_pos
        self.action_scale = action_scale

        # 加载策略模型
        self.policy = ActorCritic(
            num_actor_obs=num_obs,
            num_critic_obs=num_obs,
            num_actions=num_actions,
            actor_hidden_dims=[128, 128, 128],
            critic_hidden_dims=[128, 128, 128],
            activation='elu',
            normalize_observations=True
        )

        loaded_dict = torch.load(model_path, map_location="cpu", weights_only=True)
        self.policy.load_state_dict(loaded_dict["model_state_dict"])
        self.policy.eval()

        print(f"[PolicyRunner] 模型已加载: {model_path}")

    def infer_action(self, obs: np.ndarray) -> np.ndarray:
        """
        输入观测向量 -> 输出电机目标角度
        obs=[
                base_linear_velocity,   # 3D
                base_angular_velocity,  # 3D
                projected_gravity,      # 3D
                velocity_commands,      # 3D
                joint_positions,        # 12D
                joint_velocities,       # 12D
                last_actions,         # 12D
            ]
        :return: 1D numpy array, shape = (num_actions,)

        In Isaac Sim, the joint ordering is as follows:
        [
            'LF_HAA', 'LH_HAA', 'RF_HAA', 'RH_HAA',
            'LF_HFE', 'LH_HFE', 'RF_HFE', 'RH_HFE',
            'LF_KFE', 'LH_KFE', 'RF_KFE', 'RH_KFE'
        ]
        """
        assert obs.shape[0] == self.num_obs, \
            f"观测维度错误: 期望 {self.num_obs}, 实际 {obs.shape[0]}"

        obs_tensor = torch.from_numpy(obs.astype(np.float32)).unsqueeze(0)
        with torch.no_grad():
            raw_action = self.policy.act_inference(obs_tensor)
        raw_action_np = raw_action.squeeze(0).cpu().numpy()

        # 输出最终目标角
        target_joint_pos = self.default_joint_pos + self.action_scale * raw_action_np
        return raw_action_np,target_joint_pos


