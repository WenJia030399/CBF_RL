import gymnasium as gym
from gymnasium import spaces
import numpy as np


class UAVExploreGridEnv(gym.Env):
    """
    High-Level Step 1 Environment: Discrete Coverage Exploration
    """
    metadata = {"render_modes": ["human"]}

    def __init__(self, grid_size=(20, 20, 6), local_radius=2, max_steps=1000):
        super().__init__()

        self.grid_size = np.array(grid_size, dtype=np.int32)
        self.local_radius = int(local_radius)
        self.max_steps = int(max_steps)

        # visited grid: 0 = unknown, 1 = visited
        self.visited = np.zeros(self.grid_size, dtype=np.int8)

        # action: 6-direction discrete move
        # 0:+X, 1:-X, 2:+Y, 3:-Y, 4:+Z, 5:-Z
        self.action_space = spaces.Discrete(6)

        # observation: local visited grid + frontier density (8) + visited ratio (1)
        local_dim = (2 * self.local_radius + 1) ** 3
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(local_dim + 8 + 1,),
            dtype=np.float32,
        )

        # ---------- anti-stuck helpers ----------
        self.stay_counter = 0
        self.last_action = None

        self.reset()

    # ------------------------- Gym API -------------------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.visited.fill(0)

        # random start position
        self.pos = np.array([
            np.random.randint(1, self.grid_size[0]),
            np.random.randint(1, self.grid_size[1]),
            np.random.randint(1, self.grid_size[2]),
        ], dtype=np.int32)

        self.visited[tuple(self.pos)] = 1
        self.t = 0

        # reset helpers
        self.stay_counter = 0
        self.last_action = None

        return self._get_obs(), {}

    def step(self, action):
        self.t += 1

        # ---------- base step penalty ----------
        reward = -0.05

        # discrete moves
        moves = np.array([
            [ 1, 0, 0],  # +X
            [-1, 0, 0],  # -X
            [ 0, 1, 0],  # +Y
            [ 0,-1, 0],  # -Y
            [ 0, 0, 1],  # +Z
            [ 0, 0,-1],  # -Z
        ], dtype=np.int32)

        new_pos = self.pos + moves[int(action)]
        new_pos = np.clip(new_pos, [1, 1, 1], self.grid_size - 1)
        new_pos = new_pos.astype(np.int32)

        # ---------- exploration reward ----------
        if self.visited[tuple(new_pos)] == 0:
            reward += 1.0
            self.visited[tuple(new_pos)] = 1
        else:
            reward -= 0.3  # 比原本更重，避免重複踩格子

        # ---------- frontier attraction ----------
        frontier_after = self._count_frontiers(new_pos)
        reward += 0.1 * frontier_after

        # ---------- anti-stuck penalty ----------
        if np.all(new_pos == self.pos):
            self.stay_counter += 1
        else:
            self.stay_counter = 0

        reward -= 0.2 * self.stay_counter

        # ---------- direction consistency ----------
        if self.last_action is not None and action == self.last_action:
            reward += 0.05

        self.last_action = action

        # update state
        self.pos = new_pos

        visited_ratio = float(self.visited.mean())

        done = False
        if visited_ratio > 0.98:
            done = True
        if self.t >= self.max_steps:
            done = True

        info = {
            "visited_ratio": visited_ratio,
            "steps": self.t,
        }

        return self._get_obs(), float(reward), done, False, info

    # ------------------------- Observation -------------------------
    def _get_obs(self):
        r = self.local_radius
        x, y, z = self.pos
        local = []

        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    p = np.array([x + dx, y + dy, z + dz])
                    if np.any(p < 0) or np.any(p >= self.grid_size):
                        local.append(1.0)
                    else:
                        local.append(float(self.visited[tuple(p)]))

        local = np.asarray(local, dtype=np.float32)

        # frontier density (XY plane)
        frontier = np.zeros(8, dtype=np.float32)
        angles = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        for i, ang in enumerate(angles):
            direction = np.array([np.cos(ang), np.sin(ang), 0.0])
            step = np.round(direction).astype(np.int32)
            p = self.pos + step
            if np.all(p >= 0) and np.all(p < self.grid_size):
                if self.visited[tuple(p)] == 0:
                    frontier[i] = 1.0

        visited_ratio = np.array([self.visited.mean()], dtype=np.float32)

        return np.concatenate([local, frontier, visited_ratio])

    # ------------------------- Frontier Helper -------------------------
    def _count_frontiers(self, pos):
        count = 0
        for dx, dy, dz in [
            (1,0,0), (-1,0,0),
            (0,1,0), (0,-1,0),
            (0,0,1), (0,0,-1)
        ]:
            p = pos + np.array([dx, dy, dz])
            if np.all(p >= 0) and np.all(p < self.grid_size):
                if self.visited[tuple(p)] == 0:
                    count += 1
        return count

    def render(self):
        pass
