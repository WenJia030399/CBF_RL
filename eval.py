# eval.py
import os
from stable_baselines3 import PPO
from train import CBFWrapper, make_env as make_env_from_train  # 確保對應 train.py

def evaluate(model_path, episodes=3, render=True):
    if not os.path.exists(model_path):
        print(f"❌ Model not found: {model_path}")
        return

    print(f"Loading model: {model_path}")
    model = PPO.load(model_path)
    env = make_env_from_train(render=render)

    for ep in range(episodes):
        obs, _ = env.reset()
        done = False
        total_reward = 0.0
        step = 0

        while not done and step < 100:
            print(f"step:{step}")
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            step += 1
            done = terminated or truncated

            if render:
                env.render()  # 呼叫環境內建 render，支援多房間
            
            if info["collision"]:
                print("⚠️ Collision detected!")

        print(f"Episode {ep+1}/{episodes}: steps={step}, total_reward={total_reward:.2f}, info={info}")

    print("Evaluation done.")

if __name__ == "__main__":
    model_path = r"/home/wenjia/CBFRL/logs/ppo_cbf_1764483539/best_model.zip"
    evaluate(model_path, episodes=10, render=True)
