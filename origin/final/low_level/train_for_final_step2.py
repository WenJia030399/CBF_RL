import os, time, numpy as np
from env_for_final import UAVEnv
import gymnasium as gym
import wandb
from wandb.integration.sb3 import WandbCallback

# stable-baselines3 optional
try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
except Exception as e:
    PPO = None

try:
    from scipy.optimize import minimize
except:
    minimize = None

class CustomWandbCallback(WandbCallback):
    """
    Works for SB3 v2.x.
    Logs PPO losses into W&B.
    """
    def __init__(self):
        super().__init__()

    def _on_step(self) -> bool:
        # SB3 logger stores logs here:
        log_dict = self.model.logger.name_to_value

        # check if PPO losses exist
        if "train/policy_gradient_loss" in log_dict:
            wandb.log({
                "loss/policy": log_dict["train/policy_gradient_loss"],
                "loss/value": log_dict["train/value_loss"],
                "loss/entropy": log_dict["train/entropy_loss"],
                "loss/approx_kl": log_dict["train/approx_kl"],
                "loss/clip_fraction": log_dict["train/clip_fraction"],
                "loss/explained_variance": log_dict["train/explained_variance"],
            })

        return True

class CBFWrapper(gym.Wrapper):
    """
    Projects RL velocity action u_rl (v_x,v_y,v_z) to safe action u by
    solving min ||u-u_rl||^2 s.t. a^T u >= b  (for closest obstacle)
    with bounds.
    a = 2*(p - p_obs), b = -gamma * h_min
    """
    def __init__(self, env: UAVEnv, gamma=5.0):
        super().__init__(env)
        self.env = env
        self.gamma = gamma
        self.lb = self.env.action_space.low.copy()
        self.ub = self.env.action_space.high.copy()

    def _solve_projection(self, u_rl, pos):
        # if no scipy, fallback to clipping
        self.lb = self.lb[:3]
        self.ub = self.ub[:3]
        if minimize is None:
            return np.clip(u_rl, self.lb, self.ub)

        # find most critical obstacle
        h_min, idx = self.env.h_min_and_index(pos)
        a = self.env.dh_dv_coeff(pos, idx)  # shape (3,)
        b = - self.gamma * h_min

        # if constraint satisfied, just clip
        if float(np.dot(a, u_rl)) >= b:
            return np.clip(u_rl, self.lb, self.ub)

        # quadratic objective
        def obj(u):
            return 0.5 * np.sum((u - u_rl)**2)

        def jac(u):
            return u - u_rl

        cons = ({
            'type': 'ineq',
            'fun': lambda u, a=a, b=b: float(np.dot(a, u) - b),
            'jac': lambda u, a=a: a.astype(float)
        },)

        bounds = [(float(self.lb[i]), float(self.ub[i])) for i in range(3)]
        x0 = np.clip(u_rl, self.lb, self.ub)

        res = minimize(
            obj, x0, jac=jac,
            bounds=bounds, constraints=cons,
            method='SLSQP',
            options={'ftol': 1e-6, 'maxiter': 50, 'disp': False}
        )

        if res.success:
            return res.x.astype(np.float32)
        else:
            # analytic fallback
            a_norm2 = np.dot(a, a)
            if a_norm2 <= 1e-9:
                return np.clip(u_rl, self.lb, self.ub)
            tau = max(0.0, (b - np.dot(a, u_rl)) / a_norm2)
            u_proj = u_rl + tau * a
            return np.clip(u_proj, self.lb, self.ub)

    def step(self, action):
        u_rl = np.array(action, dtype=np.float64).flatten()
        pos = self.env.pos.copy()
        yaw_rate = u_rl[3]
        u_rl = u_rl[:3]
        u_safe = self._solve_projection(u_rl, pos)
        u_safe_full = np.concatenate([u_safe, [yaw_rate]]).astype(np.float32)
        obs, reward, done, truncated, info = self.env.step(u_safe_full)
        # obs, reward, done, truncated, info = self.env.step(u_rl)

        # Add diagnostics
        info['u_rl'] = u_rl.tolist()
        info['u_safe'] = u_safe.tolist()
        info['cbf_intervened'] = not np.allclose(u_rl, u_safe, atol=1e-6)

        # Penalty for intervention
        reward += -1.0 * np.linalg.norm(u_safe - u_rl)

        return obs, reward, done, truncated, info

    def reset(self, **kwargs):
        return self.env.reset(**kwargs)


# helper
def make_env(render=True):
    base = UAVEnv(render_mode=render)
    return CBFWrapper(base, gamma=6.0)


def rollout_random(env, episodes=2):
    for ep in range(episodes):
        obs, _ = env.reset()
        total_r = 0.0
        steps = 0
        while True:
            action = env.action_space.sample()
            obs, r, done, truncated, info = env.step(action)
            total_r += r
            steps += 1
            if done or truncated:
                print(f"Random ep {ep} steps {steps} reward {total_r:.2f} info {info}")
                break


# -------------------------------
# Main
# -------------------------------
if __name__ == "__main__":
    env = make_env(render=False)
    print("Random rollouts to check env...")
    rollout_random(env, episodes=2)

    if PPO is None:
        print("stable-baselines3 not found. Install with `pip install stable-baselines3[extra]`")
    else:
        vec_env = make_vec_env(lambda: make_env(render=False), n_envs=4)

        timestamp = int(time.time())
        logdir = f"./logs/ppo_cbf_{timestamp}"
        os.makedirs(logdir, exist_ok=True)

        # -------------------------------
        # Init W&B
        # -------------------------------
        wandb.init(
            project="ppo_cbf_uav",
            name=f"run_{timestamp}",
            config={
                "learning_rate": 1e-4,
                "n_steps": 4096,
                "batch_size": 64,
                "ent_coef": 0.01,
                "algo": "PPO",
                "cbf": True,
            }
        )

        # -------------------------------
        # Load pretrained model + Freeze layers
        # -------------------------------

        # ❗ 載入你已有的 XYZ model
        pretrained_path = "/home/wenjia/CBFRL/origin/final/low_level/logs/xyz/best_model.zip"    # 你要改成你的檔名

        model = PPO.load(pretrained_path, env=vec_env, device="cpu")
        policy = model.policy

        # 1) Freeze features extractor
        for p in policy.features_extractor.parameters():
            p.requires_grad = False
        print("[Freeze] features_extractor frozen")

        # 2) Freeze first two layers of policy MLP
        policy_layers = list(policy.mlp_extractor.policy_net.children())
        for layer in policy_layers[:2]:
            for p in layer.parameters():
                p.requires_grad = False
        print("[Freeze] first two layers of policy_net frozen")

        # (optional) 3) Freeze value_net 前兩層
        value_layers = list(policy.mlp_extractor.value_net.children())
        for layer in value_layers[:2]:
            for p in layer.parameters():
                p.requires_grad = False
        print("[Freeze] first two layers of value_net frozen")


        checkpoint_cb = CheckpointCallback(
            save_freq=1000000,
            save_path=logdir,
            name_prefix="ppo_cbf"
        )

        eval_env = make_env(render=False)
        eval_cb = EvalCallback(
            eval_env,
            best_model_save_path=logdir,
            n_eval_episodes=5,
            log_path=logdir,
            eval_freq=5000,
            deterministic=True
        )


        wandb_cb = CustomWandbCallback()

        # -------------------------------
        # Training (with wandb)
        # -------------------------------
        try:
            model.learn(
                total_timesteps=600000,
                callback=[checkpoint_cb, eval_cb, wandb_cb]
            )
        except KeyboardInterrupt:
            print("Interrupted. Saving model...")

        model.save(os.path.join(logdir, "final_model"))
        print(f"Saved model to {logdir}/final_model.zip")

        # demo with render
        demo_env = make_env(render=True)
        obs, _ = demo_env.reset()
        for _ in range(1000):
            action, _ = model.predict(obs, deterministic=True)
            obs, r, done, truncated, info = demo_env.step(action)
            demo_env.render()
            if done or truncated:
                break

        print("Demo finished.")
