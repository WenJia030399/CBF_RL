import gymnasium as gym
from gymnasium import spaces
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import matplotlib.gridspec as gridspec

try:
    from scipy.optimize import minimize
except:
    minimize = None


class UAVEnv(gym.Env):
    """
    CBF-enabled UAV Environment (drop-in replacement for env_old.py)

    Key improvements / CBF details:
    - Defines formal barrier function h(x) for cuboids (AABB) and cylinders
      as: h = distance_to_obs - (obs_margin + safety_margin).
      Positive h => safe, negative => collision.
    - Computes dh/dx (grad_h) for each obstacle type.
    - Uses a class-K function alpha(h) = k * h (linear) to form the CBF
      condition: dh/dt + alpha(h) >= 0 -> grad_h^T * v + k*h >= 0.
    - Provides helpers to return the (a, b) pair used by CBF QP projection:
         a = grad_h (so dh/dt = a^T v)
         b = - alpha(h) = -k * h

    Notes:
    - Dynamics are simple integrator: p_dot = v (action is desired velocity)
    - This file keeps the original structure and rendering from the base
      environment but fills in rigorous CBF math required by a CBF wrapper.
    """

    metadata = {"render_modes": ["human"]}

    def __init__(self, render_mode=False):
        super().__init__()

        self.room_size = 10.0  # each room 10m x 10m
        self.num_rooms = 2
        self.space_limit = 3.0
        self.dt = 0.1
        self.max_steps = 2500

        # observation: pos(3) + vel(3) + goal(3)
        high = np.array([
            self.room_size * self.num_rooms,  # x
            self.room_size,                    # y
            self.space_limit,                  # z
        ] * 3, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        # action: desired velocity
        vmax = 1.0
        self.action_space = spaces.Box(
            low=np.array([-vmax, -vmax, -vmax], dtype=np.float32),
            high=np.array([vmax, vmax, vmax], dtype=np.float32),
            dtype=np.float32
        )

        self.render_mode = render_mode
        if self.render_mode:
            self._init_render()

        # obstacles
        self.obstacles = []
        # each obstacle will have fields: type, pos, size, radius (for cylinder)
        # safety parameters
        self.safety_margin = 0.2  # added to obstacle radius/size when computing h
        self.door_height = 3.0
        self.door_width = 4.0

        # default CBF parameter (class-K gain)
        self.cbf_k = 6.0

        # initialize
        self.reset()

    # ---------------- Rendering helpers (unchanged layout) ----------------
    def _init_render(self):
        plt.close('all')
        self.fig = plt.figure(figsize=(12, 8))
        gs = gridspec.GridSpec(2, 2, figure=self.fig, width_ratios=[2, 1], height_ratios=[1, 1],
                               wspace=0.3, hspace=0.3)

        self.ax_xy = self.fig.add_subplot(gs[0, 0])
        self.ax_yz = self.fig.add_subplot(gs[0, 1])
        self.ax_xz = self.fig.add_subplot(gs[1, 0])

        plt.ion()
        plt.show()

    # ---------------- Environment construction helpers ----------------
    def _add_walls_as_obstacles(self):
        thickness = 0.2
        full_height = 3.0
        W = self.room_size * self.num_rooms
        H = self.room_size

        walls = []

        # left wall (x=0)
        walls.append({
            "type": "cuboid",
            "pos": np.array([0 + thickness / 2, H / 2, full_height / 2]),
            "size": np.array([thickness, H, full_height])
        })

        # right wall (x=W)
        walls.append({
            "type": "cuboid",
            "pos": np.array([W - thickness / 2, H / 2, full_height / 2]),
            "size": np.array([thickness, H, full_height])
        })

        # bottom wall (y=0)
        walls.append({
            "type": "cuboid",
            "pos": np.array([W / 2, 0 + thickness / 2, full_height / 2]),
            "size": np.array([W, thickness, full_height])
        })

        # top wall (y=H)
        walls.append({
            "type": "cuboid",
            "pos": np.array([W / 2, H - thickness / 2, full_height / 2]),
            "size": np.array([W, thickness, full_height])
        })

        # middle wall with door (split into two cuboids)
        mid_x = self.room_size
        door_w = self.door_width
        # upper
        walls.append({
            "type": "cuboid",
            "pos": np.array([
                mid_x,
                (H + door_w) / 2 + (H - door_w) / 4,
                full_height / 2
            ]),
            "size": np.array([thickness, (H - door_w) / 2, full_height])
        })
        # lower
        walls.append({
            "type": "cuboid",
            "pos": np.array([
                mid_x,
                (H - door_w) / 4,
                full_height / 2
            ]),
            "size": np.array([thickness, (H - door_w) / 2, full_height])
        })

        return walls

    def _generate_obstacles(self):
        self.obstacles = []
        num_obs = random.randint(3, 6)
        for _ in range(num_obs):
            shape_type = random.choice(["cuboid", "cylinder"])
            room_idx = random.randint(0, self.num_rooms - 1)
            x = random.uniform(room_idx * self.room_size + 1, (room_idx + 1) * self.room_size - 1)
            y = random.uniform(1, self.room_size - 1)
            z = random.uniform(0.5, min(3.0, self.space_limit))
            if shape_type == "cuboid":
                size = np.array([random.uniform(0.5, 2.0),
                                 random.uniform(0.5, 2.0),
                                 random.uniform(1.0, 3.0)])
                self.obstacles.append({"type": "cuboid", "pos": np.array([x, y, z], dtype=np.float32), "size": size})
            else:
                # cylinder: size = [radius, unused, height]
                size = np.array([random.uniform(0.4, 1.0), 0.0, random.uniform(1.0, 3.0)])
                self.obstacles.append({"type": "cylinder", "pos": np.array([x, y, z], dtype=np.float32), "size": size})

    # ---------------- Reset / Step ----------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._generate_obstacles()
        walls = self._add_walls_as_obstacles()
        self.obstacles.extend(walls)

        # initial state
        room_idx = random.randint(0, self.num_rooms - 1)
        self.pos = np.array([random.uniform(room_idx * self.room_size + 1, (room_idx + 1) * self.room_size - 1),
                             random.uniform(1, self.room_size - 1), 0.5], dtype=np.float32)
        self.vel = np.zeros(3, dtype=np.float32)

        # random goal not colliding with obstacles
        while True:
            room_idx_goal = random.randint(0, self.num_rooms - 1)
            gx = random.uniform(room_idx_goal * self.room_size + 1, (room_idx_goal + 1) * self.room_size - 1)
            gy = random.uniform(1, self.room_size - 1)
            gz = random.uniform(0.5, min(3.0, self.space_limit))
            goal = np.array([gx, gy, gz], dtype=np.float32)
            if all(self._h_for_obs(goal, obs) > 0.1 for obs in self.obstacles):
                break
        self.goal = goal

        self.start_pos = self.pos.copy()
        self.prev_dist = np.linalg.norm(self.goal - self.pos)
        self.t = 0
        self._history = [self.pos.copy()]

        obs = np.concatenate((self.pos, self.vel, self.goal)).astype(np.float32)
        return obs, {}

    def step(self, action):
        v_des = np.clip(action, self.action_space.low, self.action_space.high).astype(np.float32)
        alpha = 0.5
        self.vel = alpha * v_des + (1 - alpha) * self.vel
        self.pos = self.pos + self.vel * self.dt
        # clip workspace
        self.pos[0] = np.clip(self.pos[0], 0, self.num_rooms * self.room_size)
        self.pos[1] = np.clip(self.pos[1], 0, self.room_size)
        self.pos[2] = np.clip(self.pos[2], 0, self.space_limit)

        self.t += 1
        self._history.append(self.pos.copy())

        # distances
        dist_to_goal = np.linalg.norm(self.goal - self.pos)
        min_dist = self._min_dist_to_obstacles()
        collision = min_dist <= 0

        # reward shaping (keeps original style but slightly tuned)
        reward = 0.0
        progress = self.prev_dist - dist_to_goal
        reward += 5.0 * progress
        reward += -0.2 * dist_to_goal
        reward += -0.4 * abs(self.pos[2] - self.goal[2])
        reward += -0.01 * np.linalg.norm(v_des)

        # line-following reward
        p_xy = self.pos[:2]
        p0 = self.start_pos[:2]
        g_xy = self.goal[:2]
        d = g_xy - p0
        d_norm2 = np.dot(d, d) + 1e-9
        t_proj = np.dot(p_xy - p0, d) / d_norm2
        t_proj = np.clip(t_proj, 0.0, 1.0)
        closest = p0 + t_proj * d
        line_error = np.linalg.norm(p_xy - closest)
        reward += -0.5 * line_error

        if dist_to_goal < 3.0:
            reward += 3.0 * (1 - np.tanh(3 * dist_to_goal))

        done = False
        info = {"min_dist_to_obstacle": float(min_dist), "collision": collision, "line_error": float(line_error)}

        if collision:
            reward -= 200.0
            done = True
            info["collision"] = True
        elif dist_to_goal < 0.05:
            reward += 500.0
            done = True
            info["success"] = True
        elif self.t >= self.max_steps:
            done = True
            info["timeout"] = True

        self.prev_dist = dist_to_goal
        obs = np.concatenate((self.pos, self.vel, self.goal)).astype(np.float32)
        return obs, float(reward), done, False, info

    # ---------------- CBF core functions ----------------
    def _h_for_obs(self, pos, obs):
        """
        Return scalar h value for position `pos` relative to one obstacle.
        h = distance_to_obs - (obs_effective_radius + safety_margin)
        Positive => safe.
        """
        threshold = self.safety_margin
        if obs["type"] == "cuboid":
            d = self.point_to_aabb_distance(pos, obs["pos"], obs["size"])  # Euclidean distance (0 if inside)
            return float(d - threshold)
        elif obs["type"] == "cylinder":
            center = obs["pos"]
            radius = float(obs["size"][0])
            height = float(obs["size"][2])
            # horizontal distance outside radius
            d_xy = np.linalg.norm(pos[:2] - center[:2]) - radius
            half_h = height / 2.0
            dz = 0.0
            if pos[2] < center[2] - half_h:
                dz = (center[2] - half_h) - pos[2]
            elif pos[2] > center[2] + half_h:
                dz = pos[2] - (center[2] + half_h)
            d = float(np.linalg.norm([max(d_xy, 0.0), dz]))
            return float(d - threshold)
        else:
            d = float(np.linalg.norm(pos - obs["pos"]))
            return float(d - threshold)

    def h_values_state(self, pos):
        """Return array of h values for all obstacles."""
        return np.array([self._h_for_obs(pos, obs) for obs in self.obstacles], dtype=np.float32)

    def h_min_and_index(self, pos):
        hvals = self.h_values_state(pos)
        idx = int(np.argmin(hvals))
        return float(hvals[idx]), idx

    def grad_h_wrt_pos(self, pos, obs_idx):
        """
        Compute dh/dpos (3,) for obstacle obs_idx. This is used as 'a' in
        dh/dt = a^T v because dynamics are p_dot = v.
        """
        if len(self.obstacles) == 0 or obs_idx < 0 or obs_idx >= len(self.obstacles):
            return np.zeros(3, dtype=np.float32)

        obs = self.obstacles[obs_idx]
        if obs["type"] == "cuboid":
            closest = self.closest_point_on_aabb(pos, obs["pos"], obs["size"])  # closest point on/in box
            diff = pos - closest
            dist = np.linalg.norm(diff)
            if dist <= 1e-8:
                # inside or on surface: gradient is outward normal toward largest penetration axis
                # find penetration by axis
                half = 0.5 * obs["size"]
                aabb_min = obs["pos"] - half
                aabb_max = obs["pos"] + half
                # compute signed distances to faces
                ds = np.stack([pos - aabb_min, aabb_max - pos], axis=0)  # (2,3)
                # pick axis with smallest clearance
                mins = np.minimum(ds[0], ds[1])
                axis = int(np.argmin(mins))
                sign = 1.0 if (pos[axis] - obs["pos"][axis]) >= 0 else -1.0
                g = np.zeros(3, dtype=np.float32)
                g[axis] = sign
                return g
            else:
                return (diff / (dist + 1e-12)).astype(np.float32)

        elif obs["type"] == "cylinder":
            center = obs["pos"]
            radius = float(obs["size"][0])
            height = float(obs["size"][2])
            vec_xy = pos[:2] - center[:2]
            dist_xy = np.linalg.norm(vec_xy)
            half_h = height / 2.0
            # outside in z
            if pos[2] < center[2] - half_h:
                dz = (center[2] - half_h) - pos[2]
                if dist_xy > 1e-8:
                    g_xy = vec_xy / dist_xy
                    g = np.array([g_xy[0], g_xy[1], -1.0], dtype=np.float32)
                    g_norm = np.linalg.norm(g)
                    if g_norm > 1e-8:
                        return (g / g_norm).astype(np.float32)
                    else:
                        return np.array([0.0, 0.0, -1.0], dtype=np.float32)
                else:
                    return np.array([0.0, 0.0, -1.0], dtype=np.float32)
            elif pos[2] > center[2] + half_h:
                if dist_xy > 1e-8:
                    g_xy = vec_xy / dist_xy
                    g = np.array([g_xy[0], g_xy[1], 1.0], dtype=np.float32)
                    g_norm = np.linalg.norm(g)
                    if g_norm > 1e-8:
                        return (g / g_norm).astype(np.float32)
                    else:
                        return np.array([0.0, 0.0, 1.0], dtype=np.float32)
                else:
                    return np.array([0.0, 0.0, 1.0], dtype=np.float32)
            else:
                # within height band: gradient only radial in xy
                if dist_xy <= 1e-8:
                    return np.zeros(3, dtype=np.float32)
                g_xy = vec_xy / dist_xy
                return np.array([g_xy[0], g_xy[1], 0.0], dtype=np.float32)

        else:
            diff = pos - obs["pos"]
            norm = np.linalg.norm(diff)
            if norm <= 1e-8:
                return np.zeros(3, dtype=np.float32)
            return (diff / norm).astype(np.float32)

    def dh_dv_coeff(self, pos, obs_idx):
        """
        For integrator dynamics p_dot = v, dh/dt = grad_h^T * v. So dh/dv == grad_h.
        Keep this for backward compatibility with wrappers expecting dh_dv_coeff.
        """
        return self.grad_h_wrt_pos(pos, obs_idx)

    def cbf_a_b(self, pos, obs_idx=None, k=None):
        gamma = 2.0 if k is None else k

        h_min, idx = self.h_min_and_index(pos) if obs_idx is None else (None, obs_idx)
        if idx < 0 or idx >= len(self.obstacles):
            return np.zeros(3, dtype=np.float32), -1e9

        obs = self.obstacles[idx]
        obs_pos = obs["pos"].astype(np.float64)
        px, py, pz = pos

        # 以 cuboid 為例，CBF 只是取水平距離向量
        if obs["type"] == "cuboid":
            closest = self.closest_point_on_aabb(pos, obs_pos, obs["size"])
            diff = pos - closest
            a = diff.astype(np.float64)
        elif obs["type"] == "cylinder":
            center = obs_pos
            radius = float(obs["size"][0])
            vec_xy = pos[:2] - center[:2]
            a = np.array([vec_xy[0], vec_xy[1], 0.0], dtype=np.float64)
        else:
            # fallback
            a = (pos - obs_pos).astype(np.float64)

        # alpha(h)
        h_val = self._h_for_obs(pos, obs)
        b = -gamma * h_val  # alpha(h) = gamma * h

        return a, b



    def is_cbf_satisfied(self, pos, v, k=None):
        """
        Check whether the most-critical obstacle satisfies the CBF constraint under velocity v.
        """
        h_min, idx = self.h_min_and_index(pos)
        a, b = self.cbf_a_b(pos, idx, k=k)
        return float(np.dot(a, v)) >= b

    # ---------------- distances / geometry ----------------
    def point_to_aabb_distance(self, point, aabb_center, aabb_size):
        half = 0.5 * np.array(aabb_size, dtype=np.float32)
        aabb_min = aabb_center - half
        aabb_max = aabb_center + half
        dx = np.maximum(0.0, np.maximum(aabb_min - point, point - aabb_max))
        return float(np.linalg.norm(dx))

    def closest_point_on_aabb(self, point, aabb_center, aabb_size):
        half = 0.5 * np.array(aabb_size, dtype=np.float32)
        aabb_min = aabb_center - half
        aabb_max = aabb_center + half
        return np.minimum(np.maximum(point, aabb_min), aabb_max)

    def _min_dist_to_obstacles(self):
        if len(self.obstacles) == 0:
            return float('inf')
        p = self.pos
        dists = [self._h_for_obs(p, obs) + self.safety_margin for obs in self.obstacles]
        # h = d - margin. We want raw distance -> add margin back
        return float(min(dists))

    # ---------------- Rendering (keeps original look) ----------------
    def render(self):
        if not self.render_mode:
            return

        if not hasattr(self, "ax_xy"):
            plt.close(self.fig)
            self.fig = plt.figure(figsize=(12, 8))
            gs = gridspec.GridSpec(2, 2, figure=self.fig, width_ratios=[2, 1], height_ratios=[1, 1],
                                   wspace=0.3, hspace=0.3)
            self.ax_xy = self.fig.add_subplot(gs[0, 0])
            self.ax_yz = self.fig.add_subplot(gs[0, 1])
            self.ax_xz = self.fig.add_subplot(gs[1, 0])
            plt.ion()
            plt.show()

        ax_xy = self.ax_xy
        ax_xz = self.ax_xz
        ax_yz = self.ax_yz

        ax_xy.clear(); ax_xz.clear(); ax_yz.clear()

        for i in range(self.num_rooms):
            ax_xy.add_patch(Rectangle((i * self.room_size, 0), self.room_size, self.room_size,
                                      fill=False, edgecolor='gray'))

        for obs in self.obstacles:
            x, y, z = obs["pos"]
            if obs["type"] == "cuboid":
                sx, sy, sz = obs["size"]
                ax_xy.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='red', alpha=0.4))
                ax_xz.add_patch(Rectangle((x - sx / 2, z - sz / 2), sx, sz, color='red', alpha=0.3))
                ax_yz.add_patch(Rectangle((y - sy / 2, z - sz / 2), sy, sz, color='red', alpha=0.3))
            else:
                radius, _, height = obs["size"]
                ax_xy.add_patch(Circle((x, y), radius, color='red', alpha=0.4))
                ax_xz.add_patch(Rectangle((x - radius, z - height / 2), radius * 2, height, color='red', alpha=0.3))
                ax_yz.add_patch(Rectangle((y - radius, z - height / 2), radius * 2, height, color='red', alpha=0.3))

        sx, sy = self.start_pos[:2]
        gx, gy = self.goal[:2]
        ax_xy.plot([sx, gx], [sy, gy], '--k', alpha=0.5)

        hist = np.array(self._history)
        if hist.shape[0] > 0:
            ax_xy.plot(hist[:, 0], hist[:, 1], '-b')
            ax_xz.plot(hist[:, 0], hist[:, 2], '-b')
            ax_yz.plot(hist[:, 1], hist[:, 2], '-b')

        x, y, z = self.pos
        ax_xy.set_aspect('equal', adjustable='box')
        ax_xy.plot(x, y, 'bo')
        ax_xy.plot(self.goal[0], self.goal[1], 'gx')

        ax_xz.set_aspect('equal', adjustable='box')
        ax_xz.plot(x, z, 'bo')
        ax_xz.plot(self.goal[0], self.goal[2], 'gx')

        ax_yz.set_aspect('equal', adjustable='box')
        ax_yz.plot(y, z, 'bo')
        ax_yz.plot(self.goal[1], self.goal[2], 'gx')

        ax_xy.set_xlim(0, self.num_rooms * self.room_size)
        ax_xy.set_ylim(0, self.room_size)
        ax_xy.set_title(f"XY (t={self.t})")

        ax_xz.set_xlim(0, self.num_rooms * self.room_size)
        ax_xz.set_ylim(0, self.space_limit)
        ax_xz.set_title("XZ")

        ax_yz.set_xlim(0, self.room_size)
        ax_yz.set_ylim(0, self.space_limit)
        ax_yz.set_title("YZ")

        plt.pause(0.001)
