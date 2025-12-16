# ------------------------- evaluation.py -------------------------
import rclpy
from env_gazebo import UAVEnvCBF
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
import numpy as np

def evaluate(model_path="/home/wenjia/CBFRL/logs/ppo_cbf_1764483539/best_model.zip", episodes=5, render=False, action_ratio = 1):
    rclpy.init()

    print(f"\nLoading model: {model_path}")
    model = PPO.load(model_path)

    # 使用與訓練相同的 VecEnv 包裝
    env = DummyVecEnv([lambda: UAVEnvCBF(render_mode=render)])

    all_rewards = []

    for ep in range(episodes):
        obs = env.reset()
        terminated = False
        truncated = False
        episode_reward = 0

        print(f"\n===== Episode {ep + 1} / {episodes} =====")
        t = 0
        while not (terminated or truncated) and t < 750:
            print(f"Step: {t}")
            t += 1
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, dones, info = env.step(action*action_ratio)

            episode_reward += float(np.mean(reward))

            # print(f"Pos={pos}, Vel={vel}, Goal={goal}, Reward={reward}")

        print(f"Episode {ep + 1} Total Reward = {episode_reward:.2f}")
        all_rewards.append(episode_reward)

    print("\n======== Evaluation Summary ========")
    print(f"Average Reward over {episodes} episodes: {np.mean(all_rewards):.2f}")
    print(f"Std: {np.std(all_rewards):.2f}")

    rclpy.shutdown()


if __name__ == "__main__":
    evaluate(model_path="/home/wenjia/CBFRL/origin/final/logs/ppo_cbf_1765601078/best_model.zip", action_ratio=0.3)
    # evaluate(action_ratio=0.25)
