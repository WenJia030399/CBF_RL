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
        # 必須加這行！讓其他 callback（包含 WandbCallback）繼續正常運作
        super()._on_step()

        infos = self.locals.get("infos")[0]

        if isinstance(infos, dict):
            if "episode" in infos:
                wandb.log({
                    "rollout/episode_length": infos["episode"]["l"],
                    "rollout/episode_reward": infos["episode"]["r"],
                }, step=self.num_timesteps)

            if "dist_to_goal" in infos:
                wandb.log({"env/dist_to_goal": infos["dist_to_goal"]}, step=self.num_timesteps)

            if "min_dist_to_obstacle" in infos:
                wandb.log({"env/min_obstacle_dist": infos["min_dist_to_obstacle"]}, step=self.num_timesteps)

            if infos.get("collision") is True:
                wandb.log({"env/collision_count": 1}, step=self.num_timesteps)

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

    # ---- Load pretrained model if exists ----
    model_path = os.path.join("models", "/home/wenjia/CBFRL/logs/ppo_cbf_1765302852/best_model.zip")
    if os.path.exists(model_path):
        print("Loading pretrained model...")
        model = PPO.load(model_path, env=env, device="cpu")
        # 可選：更新 optimizer learning rate
        model.learning_rate = wandb.config.learning_rate
    else:
        print("Training new model from scratch...")
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
                verbose=1
            ),
            GoalLoggingCallback()
        ],
        progress_bar=False
    )

    # ---- Save model ----
    save_path = os.path.join(os.getcwd(), "models", "ppo_uav_cbf_goal")
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
