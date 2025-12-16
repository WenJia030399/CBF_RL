import time
import numpy as np
from stable_baselines3 import PPO

from low_level.env_for_final import UAVEnv
from low_level.train_for_final_step1 import CBFWrapper          # ← 把你那支 wrapper 存成 cbf_wrapper.py
from high_level.UAVExploreGridEnv import UAVExploreGridEnv


class UAVHierarchicalEnv:
    def __init__(
        self,
        grid_size=(19, 9, 3),
        local_radius=2,
        max_steps_high=100,
        max_steps_low=25,
        render=True,
        low_level_scale=1.0,
    ):
        # ---------- High-level ----------
        self.high_env = UAVExploreGridEnv(
            grid_size=grid_size,
            local_radius=local_radius,
            max_steps=max_steps_high
        )

        # ---------- Low-level (CBF wrapped) ----------
        base_low_env = UAVEnv(render_mode=render)
        self.low_env = CBFWrapper(base_low_env, gamma=6.0)

        self.grid_size = np.array(grid_size)
        self.max_steps_low = max_steps_low
        self.low_level_scale = low_level_scale
        self.render_mode = render

    def reset(self):
        high_obs, _ = self.high_env.reset()

        # sync low-level state to high-level grid pos
        grid_pos = self.high_env.pos.astype(np.float32)
        world_pos = grid_pos * self.low_level_scale

        self.low_env.env.pos = world_pos.copy()
        self.low_env.env.vel = np.zeros(3, dtype=np.float32)
        self.low_env.env._history = [world_pos.copy()]
        self.low_env.env.start_pos = world_pos.copy()
        self.low_env.env.goal = world_pos.copy()
        self.low_env.env.goal_yaw = 0.0
        self.low_env.env.t = 0

        return high_obs

    def step(self, high_action, low_policy):
        # ---------- High-level step ----------
        high_obs, reward_h, done_h, _, info_h = self.high_env.step(high_action)

        target_grid = self.high_env.pos.astype(np.float32)
        target_pos = target_grid * self.low_level_scale
        self.low_env.env.goal = target_pos.copy()

        # ---------- Low-level rollout ----------
        low_reward_total = 0.0
        done_low = False
        step_count = 0

        while not done_low and step_count < self.max_steps_low:
            step_count += 1

            pos = self.low_env.env.pos
            delta = pos - target_pos

            obs_low = np.array([
                delta[0],
                delta[1],
                delta[2],
                np.linalg.norm(delta),
                self.low_env.env.yaw,
                0.0,                         # yaw error (placeholder)
            ], dtype=np.float32)

            action_low, _ = low_policy.predict(obs_low, deterministic=True)

            obs_l, reward_l, done_l, truncated_l, info_l = self.low_env.step(action_low)
            low_reward_total += reward_l

            if self.render_mode:
                self.low_env.env.render()
                time.sleep(0.02)

            # success / collision termination
            if np.linalg.norm(self.low_env.env.pos - target_pos) < 0.2:
                done_low = True

            if info_l.get("collision", False):
                done_low = True
                reward_h -= 5.0   # ⛔ collision directly penalize high-level

            if done_l or truncated_l:
                done_low = True

        total_reward = reward_h + low_reward_total
        info_h["low_steps"] = step_count

        return high_obs, total_reward, done_h, info_h


# === 執行範例 ===
if __name__ == "__main__":
    # 載入已訓練的模型
    low_level_model_path = r"/home/wenjia/CBFRL/origin/final/logs/ppo_cbf_1765601078/best_model.zip"
    high_level_model_path = r"/home/wenjia/CBFRL/origin/final/logs/step1_explore/best/best_model.zip"

    # 載入 low-level PPO
    low_policy = PPO.load(low_level_model_path)

    # 載入 high-level PPO
    high_policy = PPO.load(high_level_model_path)

    # 初始化 hierarchical env
    env = UAVHierarchicalEnv(render=True)

    obs = env.reset()
    done = False
    total_reward = 0.0

    while not done:
        # high-level policy 選動作
        high_action, _ = high_policy.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(high_action, low_policy)
        total_reward += reward
        print(f"High Step: {env.high_env.t}, Total Reward: {total_reward:.2f}, Visited: {info.get('visited_ratio', 0.0):.2f}")

    print("Episode finished")
