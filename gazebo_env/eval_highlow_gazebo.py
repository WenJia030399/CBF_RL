# eval_highlow_gazebo.py
import sys
import os
import time
import numpy as np
import rclpy
from stable_baselines3 import PPO
import matplotlib.pyplot as plt

# ----------------- 修正 import 路徑 -----------------
current_dir = os.path.dirname(os.path.abspath(__file__))  # ~/CBFRL/gazebo_env
cbfrl_dir = os.path.dirname(current_dir)                # ~/CBFRL
origin_final_dir = os.path.join(cbfrl_dir, "origin", "final")
gazebo_env_dir = current_dir  # gazebo_env 本身

# 加入 sys.path
for p in [origin_final_dir, gazebo_env_dir]:
    if p not in sys.path:
        sys.path.append(p)

from high_level.UAVExploreGridEnv import UAVExploreGridEnv
from env_gazebo import UAVEnvCBF  # 這裡直接用相對於 gazebo_env 的 import


class UAVHierarchicalGazeboEnv:
    def __init__(
        self,
        grid_size=(19, 9, 3),
        local_radius=2,
        max_steps_high=2500,
        max_steps_low=250,
        low_level_scale=1.0,
        render=True,
    ):
        self.traj = []        # low-level 實際走過的軌跡
        self.goal_traj = []   # 每次 low-level rollout 對應的 goal

        # ---------- High-level ----------
        self.high_env = UAVExploreGridEnv(
            grid_size=grid_size,
            local_radius=local_radius,
            max_steps=max_steps_high
        )

        # ---------- Low-level (Gazebo + CBF) ----------
        self.low_env = UAVEnvCBF(render_mode=render)

        self.max_steps_low = max_steps_low
        self.low_level_scale = low_level_scale
        self.render = render

    def reset(self):
        high_obs, _ = self.high_env.reset()
        start_pos = self.high_env.pos.astype(np.float32) * self.low_level_scale
        self.low_env.pos = start_pos.copy()

        # reset gazebo env
        self.low_env.start_pos = start_pos.copy()
        low_obs, _ = self.low_env.reset()

        return high_obs

    def step(self, high_action, low_policy, action_ratio=1.0):
        # ---------- High-level ----------
        high_obs, reward_h, done_h, _, info_h = self.high_env.step(high_action)

        # grid → world goal
        target_grid = self.high_env.pos.astype(np.float32)
        target_pos = target_grid * self.low_level_scale
        print(f"High-level selected target pos: {target_pos}")

        # set Gazebo goal
        self.low_env.goal = target_pos.copy()
        self.low_env.goal_yaw = 0.0

        # ---------- Low-level rollout ----------
        low_reward_total = 0.0
        step_count = 0
        done_low = False


        while not done_low and step_count < self.max_steps_low:
            step_count += 1

            d_pos = self.low_env.pos - self.low_env.goal
            goal_yaw = np.arctan2(d_pos[1], d_pos[0])
            yaw_e = self.low_env.yaw - goal_yaw
            yaw_e = np.arctan2(np.sin(yaw_e), np.cos(yaw_e))
            # low-level obs is already provided by env
            obs_low = np.array([
                *(d_pos),
                np.linalg.norm(d_pos),
                self.low_env.yaw,
                yaw_e
            ], dtype=np.float32)

            action_low, _ = low_policy.predict(obs_low, deterministic=True)
            obs_l, reward_l, done_l, _, info_l = self.low_env.step(action_low*action_ratio)
            self.traj.append(self.low_env.pos.copy())
            self.goal_traj.append(self.low_env.goal.copy())
            low_reward_total += reward_l

            if info_l.get("collision", False):
                reward_h -= 10.0
                done_low = True

            if done_l:
                done_low = True
        obs_l, reward_l, done_l, _, info_l = self.low_env.step(np.zeros(4))
        info_h["low_steps"] = step_count
        total_reward = reward_h + low_reward_total
        # 1. 同步 high-level pos
        self.high_env.pos = (self.low_env.pos / self.low_level_scale).astype(int)  

        # 2. 更新 high-level obs
        high_obs, _ = self.high_env._get_obs(), {}


        return high_obs, total_reward, done_h, info_h

def plot_trajectory_with_goals_3view(traj, goal_traj):
    traj = np.array(traj)
    goal_traj = np.array(goal_traj)

    x, y = traj[:, 0], traj[:, 1]
    z = traj[:, 2] if traj.shape[1] > 2 else np.zeros_like(x)

    gx, gy = goal_traj[:, 0], goal_traj[:, 1]
    gz = goal_traj[:, 2] if goal_traj.shape[1] > 2 else np.zeros_like(gx)

    fig, axs = plt.subplots(1, 3, figsize=(15, 4))

    # ---------- XY ----------
    axs[0].plot(x, y, '-', linewidth=2, label="Trajectory")
    axs[0].scatter(gx, gy, c='r', marker='x', s=30, label="Goals")
    axs[0].set_title("XY view (Top)")
    axs[0].set_xlabel("X")
    axs[0].set_ylabel("Y")
    axs[0].axis("equal")
    axs[0].grid(True)
    axs[0].legend()

    # ---------- XZ ----------
    axs[1].plot(x, z, '-', linewidth=2, label="Trajectory")
    axs[1].scatter(gx, gz, c='r', marker='x', s=30, label="Goals")
    axs[1].set_title("XZ view (Side)")
    axs[1].set_xlabel("X")
    axs[1].set_ylabel("Z")
    axs[1].axis("equal")
    axs[1].grid(True)

    # ---------- YZ ----------
    axs[2].plot(y, z, '-', linewidth=2, label="Trajectory")
    axs[2].scatter(gy, gz, c='r', marker='x', s=30, label="Goals")
    axs[2].set_title("YZ view (Side)")
    axs[2].set_xlabel("Y")
    axs[2].set_ylabel("Z")
    axs[2].axis("equal")
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    rclpy.init()

    low_model_path = "/home/wenjia/CBFRL/origin/final/logs/ppo_cbf_1765601078/best_model.zip"
    high_model_path = "/home/wenjia/CBFRL/origin/final/high_level/logs/step1_explore/best/best_model.zip"

    low_policy = PPO.load(low_model_path)
    high_policy = PPO.load(high_model_path)

    env = UAVHierarchicalGazeboEnv(render=True)

    obs = env.reset()
    done = False
    total_reward = 0.0

    while not done:
        high_action, _ = high_policy.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(high_action, low_policy, action_ratio=0.15)
        total_reward += reward

        print(
            f"[HighStep {env.high_env.t}] "
            f"HighAction={high_action} "
            f"Visited={info.get('visited_ratio', 0):.2f}"
        )

    print("Gazebo hierarchical evaluation finished")
    rclpy.shutdown()
    
    plot_trajectory_with_goals_3view(env.traj, env.goal_traj)
