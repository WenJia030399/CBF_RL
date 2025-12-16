import os
import numpy as np
import gymnasium as gym

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback

from UAVExploreGridEnv import UAVExploreGridEnv


# ---------------------------
# Environment Factory
# ---------------------------
def make_env(rank, seed=0):
    def _init():
        env = UAVExploreGridEnv(
            grid_size=(20, 20, 6),
            local_radius=2,
            max_steps=800,
        )
        env = Monitor(env)
        env.reset(seed=seed + rank)
        return env
    return _init


# ---------------------------
# Main Training
# ---------------------------
def main():
    log_dir = "./logs/step1_explore"
    os.makedirs(log_dir, exist_ok=True)

    n_envs = 8

    # parallel envs
    env = SubprocVecEnv([make_env(i) for i in range(n_envs)])

    # evaluation env
    eval_env = DummyVecEnv([make_env(999)])

    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,

        # ---------- PPO core ----------
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=512,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,

        # ---------- Exploration ----------
        ent_coef=0.01,
        clip_range=0.2,

        # ---------- Stability ----------
        max_grad_norm=0.5,

        tensorboard_log=log_dir,
    )

    # callbacks
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(log_dir, "best"),
        log_path=log_dir,
        eval_freq=20_000,
        n_eval_episodes=5,
        deterministic=True,
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=50_000,
        save_path=os.path.join(log_dir, "ckpt"),
        name_prefix="ppo_explore",
    )

    # ---------- Train ----------
    model.learn(
        total_timesteps=3_000_000,
        callback=[eval_callback, checkpoint_callback],
    )

    model.save(os.path.join(log_dir, "final_model"))

    env.close()
    eval_env.close()


if __name__ == "__main__":
    main()
