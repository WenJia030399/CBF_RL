import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

from stable_baselines3 import PPO
from UAVExploreGridEnv import UAVExploreGridEnv


def evaluate(model_path, n_episodes=3):
    env = UAVExploreGridEnv(grid_size=(20, 10, 3), local_radius=2, max_steps=800)
    model = PPO.load(model_path)

    for ep in range(n_episodes):
        obs, _ = env.reset()
        done = False
        trajectory = [env.pos.copy()]
        visited_history = [env.visited.copy()]
        t = 0
        while not done :
            t+=1
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            trajectory.append(env.pos.copy())
            visited_history.append(env.visited.copy())

        trajectory = np.array(trajectory)
        print(len(trajectory))

        # --------- Visualization ---------
        fig, (ax_xy, ax_xz, ax_yz) = plt.subplots(1, 3, figsize=(18, 6))
        fig.suptitle(f"Evaluation Episode {ep+1}")

        # XY plane
        ax_xy.set_title("XY plane")
        ax_xy.set_xlim(0, env.grid_size[0])
        ax_xy.set_ylim(0, env.grid_size[1])
        ax_xy.set_aspect("equal")
        ax_xy.plot(trajectory[:,0], trajectory[:,1], '-b', label="Trajectory")
        ax_xy.scatter(trajectory[0,0], trajectory[0,1], c='green', label='Start')
        ax_xy.scatter(trajectory[-1,0], trajectory[-1,1], c='red', label='End')

        # XZ plane
        ax_xz.set_title("XZ plane")
        ax_xz.set_xlim(0, env.grid_size[0])
        ax_xz.set_ylim(0, env.grid_size[2])
        ax_xz.set_aspect("equal")
        ax_xz.plot(trajectory[:,0], trajectory[:,2], '-b')

        # YZ plane
        ax_yz.set_title("YZ plane")
        ax_yz.set_xlim(0, env.grid_size[1])
        ax_yz.set_ylim(0, env.grid_size[2])
        ax_yz.set_aspect("equal")
        ax_yz.plot(trajectory[:,1], trajectory[:,2], '-b')

        # --------- Visited Grid Heatmap on XY ---------
        visited_total = visited_history[-1]
        for x in range(env.grid_size[0]):
            for y in range(env.grid_size[1]):
                if np.any(visited_total[x,y,:]):  # visited any z
                    ax_xy.add_patch(Rectangle((x, y), 1, 1, color='orange', alpha=0.3))

        ax_xy.legend()
        plt.show()
        print(f"Episode {ep+1} completed. Total Reward: {info['visited_ratio']*100:.2f}% area visited.")


if __name__ == "__main__":
    model_path = "/home/wenjia/CBFRL/origin/final/high_level/logs/step1_explore/best/best_model.zip"  # 替換成你的 model 路徑
    evaluate(model_path, n_episodes=3)
