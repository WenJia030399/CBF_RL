# ------------------------- train.py -------------------------
import rclpy
import os
from env_gazebo import UAVEnvCBF

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv

import wandb
from wandb.integration.sb3 import WandbCallback


# ------------------------- Logging Callback -------------------------
class GoalLoggingCallback(BaseCallback):
    def _on_step(self) -> bool:
        infos = self.locals.get("infos")[0]  # Get info from the first environment

        if isinstance(infos, dict):
            # 確保鍵存在，否則會崩潰 (特別是對於 DummyVecEnv)

            # 成功或超時時，infos 會包含 'episode' 字典
            if "episode" in infos:
                # 記錄回合長度和最終獎勵 (如果需要)
                wandb.log({"rollout/episode_length": infos["episode"]["l"],
                           "rollout/episode_reward": infos["episode"]["r"]},
                          step=self.num_timesteps)

            # 記錄環境特定指標
            if "dist_to_goal" in infos:
                wandb.log({"env/dist_to_goal": infos["dist_to_goal"]}, step=self.num_timesteps)

            if "min_dist_to_obstacle" in infos:
                wandb.log({"env/min_obstacle_dist": infos["min_dist_to_obstacle"]}, step=self.num_timesteps)

            # 記錄碰撞作為布林值或計數器
            if infos.get("collision") is True:
                 wandb.log({"env/collision_count": 1}, step=self.num_timesteps)
                 # 可以在此處記錄碰撞發生時的步數等資訊
                 
        return True



# ------------------------- Main Training -------------------------
def main():

    rclpy.init()

    # ---- WandB Init ----
    wandb.init(
        project="uav-cbf-rl",
        name="ppo-gazebo-run",
        config={
            "policy": "MlpPolicy",
            "learning_rate": 5e-4,
            "gamma": 0.99,
            "n_steps": 1024,
            "batch_size": 512,
        }
    )

    # ---- Env ----
    env = DummyVecEnv([lambda: UAVEnvCBF(render_mode=False)])

    # ---- PPO ----
    model = PPO(
        wandb.config.policy,
        env,
        verbose=1,
        learning_rate=wandb.config.learning_rate,
        gamma=wandb.config.gamma,
        n_steps=wandb.config.n_steps,
        batch_size=wandb.config.batch_size,
        policy_kwargs=dict(log_std_init=-1.0, ortho_init=False),
        device="cpu"
    )

    # ---- Train ----
    model.learn(
        total_timesteps=1_000_000,
        callback=[
            WandbCallback(
                gradient_save_freq=1000,
                model_save_path="models/",
                model_save_freq=10_000,
                verbose=2
            ),
            GoalLoggingCallback()
        ]
    )

    # ---- Save model ----
    save_path = os.path.join(os.getcwd(), "ppo_uav_cbf_goal")
    model.save(save_path)
    print(f"\nModel saved to: {save_path}\n")

    # ------------------------- Testing -------------------------
    print("======== Testing Policy ========")

    obs = env.reset()
    done = False

    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, rewards, dones, infos = env.step(action)
        done = dones[0]

    print("Test finished.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
