# ------------------------- train.py -------------------------
import rclpy
from env_gazebo import UAVEnvCBF
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
import os
from stable_baselines3.common.vec_env import DummyVecEnv

# ------------------------- Logging Callback -------------------------
class GoalLoggingCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)

    def _on_step(self) -> bool:
        info = self.locals.get("infos", [{}])[-1]
        if isinstance(info, dict):
            if "dist_to_goal" in info:
                self.logger.record("env/dist_to_goal", info["dist_to_goal"])
            if "crashed" in info:
                self.logger.record("env/crashed", float(info["crashed"]))
        return True

# ------------------------- Main Training -------------------------
def main():
    rclpy.init()

    # 建立環境
    # env = UAVEnvCBF(render_mode=False)
    env = DummyVecEnv([lambda: UAVEnvCBF(render_mode=False)])

    # 建立 PPO 模型
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        batch_size=64,
        n_steps=1024, # 減少以加快更新
        learning_rate=3e-4,
        gamma=0.99,
        # 增加 policy_kwargs 來優化連續動作空間的探索
        policy_kwargs=dict(log_std_init=-1.0, ortho_init=False) ,
        device='cpu'
    )

    # 訓練
    timesteps = 1000000
    model.learn(total_timesteps=timesteps, callback=GoalLoggingCallback())

    # 儲存模型
    save_path = os.path.join(os.getcwd(), "ppo_uav_cbf_goal")
    model.save(save_path)
    print(f"\nModel saved to: {save_path}\n")

    # ------------------------- Testing -------------------------
    print("======== Testing Policy ========")

    obs, _ = env.reset()
    done = False
    terminated = False
    truncated = False

    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)

        # print(
        #     f"Pos: {obs[:3]}, Goal: {obs[3:]}, "
        #     f"Reward: {reward:.3f}, Terminated: {terminated}, Truncated: {truncated}"
        # )

    print("Test finished.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
