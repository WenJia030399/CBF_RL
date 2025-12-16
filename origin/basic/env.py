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
    UAV Environment for multi-room indoor navigation
    - Action: [v_x, v_y, v_z] in world-frame
    - Observation: [x, y, z, vx, vy, vz, vector to obstacle(x,y,z),gx, gy, gz]
    - Obstacles: random cuboids + cylinders
    - CBF helpers included
    """
    metadata = {"render_modes": ["human"]}

    def __init__(self, render_mode=False):
        super().__init__()

        self.room_size = 10.0  # each room 10m x 10m
        self.num_rooms = 2
        self.space_limit = 3
        self.dt = 0.1
        self.max_steps = 2500

        # observation: pos(3) + vel(3) + goal(3)
        high = np.array([
            20, 10, 3,     # pos
            2, 2, 2,       # vel
            10,10,10,
            20, 10, 3      # goal
        ], dtype=np.float32)

        low = np.array([
            0, 0, 0,       # pos
            -2, -2, -2,    # vel
            -10,-10,-10,
            0, 0, 0        # goal
        ], dtype=np.float32)

        self.observation_space = spaces.Box(low, high, dtype=np.float32)



        # action: desired velocity
        vmax = 0.25
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
        self.safety_margin = 0.1
        self.door_height = 3.0
        self.door_width = 4.0

        self.obstacle_vector = np.zeros(3)


        self.reset()

    def _init_render(self):
        import matplotlib.gridspec as gridspec

        plt.close('all')
        self.fig = plt.figure(figsize=(12, 8))
        
        # 使用 GridSpec 安排 XY 水平，XZ 垂直，YZ 水平
        gs = gridspec.GridSpec(2, 2, figure=self.fig, width_ratios=[2,1], height_ratios=[1,1], wspace=0.3, hspace=0.3)

        self.ax_xy = self.fig.add_subplot(gs[0,0])
        self.ax_yz = self.fig.add_subplot(gs[0,1])
        self.ax_xz = self.fig.add_subplot(gs[1,0])

        plt.ion()
        plt.show()


    def _add_walls_as_obstacles(self):
        thickness = 0.2
        full_height = 3.0
        W = self.room_size * self.num_rooms   # e.g. = 20
        H = self.room_size                    # e.g. = 10

        walls = []

        # -----------------------------
        # ① 外牆（全部封閉）
        # -----------------------------

        # left wall (x=0)
        walls.append({
            "type": "cuboid",
            "pos": np.array([0 + thickness/2, H/2, full_height/2]),
            "size": np.array([thickness, H, full_height])
        })

        # right wall (x=W)
        walls.append({
            "type": "cuboid",
            "pos": np.array([W - thickness/2, H/2, full_height/2]),
            "size": np.array([thickness, H, full_height])
        })

        # bottom wall (y=0)
        walls.append({
            "type": "cuboid",
            "pos": np.array([W/2, 0 + thickness/2, full_height/2]),
            "size": np.array([W, thickness, full_height])
        })

        # top wall (y=H)
        walls.append({
            "type": "cuboid",
            "pos": np.array([W/2, H - thickness/2, full_height/2]),
            "size": np.array([W, thickness, full_height])
        })

        # -----------------------------
        # ② 中間牆（保留門洞）
        # -----------------------------
        mid_x = self.room_size  # x = 10
        door_w = self.door_width  # e.g., 2.0

        # 上半部牆
        walls.append({
            "type": "cuboid",
            "pos": np.array([
                mid_x,
                (H - door_w)/2+door_w+(H - door_w)/4,    # 正中心 + 半個門洞
                full_height/2
            ]),
            "size": np.array([
                thickness,
                (H- door_w)/2,        # 上牆長度 = 總高度 - 門洞
                full_height
            ])
        })

        # 下半部牆
        walls.append({
            "type": "cuboid",
            "pos": np.array([
                mid_x,
                (H - door_w)/4,    # 正中心 - 半個門洞
                full_height/2
            ]),
            "size": np.array([
                thickness,
                (H- door_w)/2,
                full_height
            ])
        })

        return walls


    def _generate_obstacles(self):
        self.obstacles = []
        num_obs = random.randint(3, 6)
        for _ in range(num_obs):
            shape_type = random.choice(["cuboid", "cylinder"])
            # position inside rooms
            room_idx = random.randint(0, self.num_rooms-1)
            x = random.uniform(room_idx*self.room_size + 1, (room_idx+1)*self.room_size - 1)
            y = random.uniform(1, self.room_size - 1)
            z = random.uniform(0.5, 3.0)
            # size
            if shape_type == "cuboid":
                size = np.array([random.uniform(0.5, 2.0),
                                 random.uniform(0.5, 2.0),
                                 random.uniform(1.0, 3.0)])
            else:  # cylinder
                size = np.array([random.uniform(0.5, 1.0),
                                 random.uniform(0.5, 1.0),
                                 random.uniform(1.0, 3.0)])  # radius_x, radius_y, height
            self.obstacles.append({
                "type": shape_type,
                "pos": np.array([x,y,z], dtype=np.float32),
                "size": size
            })

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # generate obstacles
        self._generate_obstacles()
        walls = self._add_walls_as_obstacles()
        self.obstacles.extend(walls)

        # initial pos + vel
        room_idx = random.randint(0, self.num_rooms-1)
        self.pos = np.array([random.uniform(room_idx*self.room_size + 1, (room_idx+1)*self.room_size - 1),
                             random.uniform(1, self.room_size - 1),
                             0.5], dtype=np.float32)
        self.vel = np.zeros(3, dtype=np.float32)
        self.obstacle_vector = np.zeros(3, dtype=np.float32)

        # random goal in other room (or same room)
        while True:
            room_idx_goal = random.randint(0, self.num_rooms-1)
            gx = random.uniform(room_idx_goal*self.room_size + 1, (room_idx_goal+1)*self.room_size - 1)
            gy = random.uniform(1, self.room_size - 1)
            gz = random.uniform(0.5, 3.0)
            goal = np.array([gx, gy, gz], dtype=np.float32)
            # avoid collision with obstacles
            collision_free = True
            for obs in self.obstacles:
                if np.linalg.norm(goal - obs["pos"]) < (self.safety_margin + 0.5):
                    collision_free = False
                    break
            if collision_free:
                break
        self.goal = goal

        self.start_pos = self.pos.copy()
        self.prev_dist = np.linalg.norm(self.goal - self.pos)
        self.t = 0
        self._history = [self.pos.copy()]

        obs = np.concatenate((self.pos, self.vel, self.obstacle_vector, self.goal)).astype(np.float32)
        return obs, {}

    def step(self, action):
        v_des = np.clip(action, self.action_space.low, self.action_space.high).astype(np.float32)
        alpha = 0.5
        self.vel = alpha*v_des + (1-alpha)*self.vel
        self.pos = self.pos + self.vel * self.dt
        self.pos[0] = np.clip(self.pos[0], 0, self.num_rooms*self.room_size)
        self.pos[1] = np.clip(self.pos[1], 0, self.room_size)
        self.pos[2] = np.clip(self.pos[2], 0, self.space_limit)

        self.t += 1
        self._history.append(self.pos.copy())

        # distances
        dist_to_goal = np.linalg.norm(self.goal - self.pos)
        min_dist, _, self.obstacle_vector = self._min_dist_to_obstacles()
        collision = min_dist <= 0

        # reward
        reward = 0.0
        progress = self.prev_dist - dist_to_goal
        reward += 5.0*progress
        reward += -0.2*dist_to_goal
        reward += -0.4*abs(self.pos[2]-self.goal[2])
        reward += -0.01*np.linalg.norm(v_des)

        # line-following reward
        p_xy = self.pos[:2]
        p0 = self.start_pos[:2]
        g_xy = self.goal[:2]
        d = g_xy - p0
        d_norm2 = np.dot(d,d)+1e-9
        t_proj = np.dot(p_xy - p0, d)/d_norm2
        t_proj = np.clip(t_proj, 0.0, 1.0)
        closest = p0 + t_proj*d
        line_error = np.linalg.norm(p_xy - closest)
        reward += -0.5*line_error

        if dist_to_goal < 3.0:
            reward += 3.0*(1 - np.tanh(3*dist_to_goal))

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
        obs = np.concatenate((self.pos, self.vel, self.obstacle_vector, self.goal)).astype(np.float32)
        return obs, float(reward), done, False, info

    # ---------------- CBF helpers ----------------
    def point_to_aabb_distance(self, point, aabb_center, aabb_size):
        """
        point: (3,)
        aabb_center: (3,) center of cuboid
        aabb_size: (3,) full lengths along x,y,z
        return: Euclidean distance from point to AABB (0 if inside)
        """
        half = 0.5 * np.array(aabb_size, dtype=np.float32)
        aabb_min = aabb_center - half
        aabb_max = aabb_center + half
        # for each axis compute distance outside the interval
        dx = np.maximum(0.0, np.maximum(aabb_min - point, point - aabb_max))
        return float(np.linalg.norm(dx))

    def closest_point_on_aabb(self, point, aabb_center, aabb_size):
        """
        Return the closest point on (or inside) the AABB to `point`.
        """
        half = 0.5 * np.array(aabb_size, dtype=np.float32)
        aabb_min = aabb_center - half
        aabb_max = aabb_center + half
        return np.minimum(np.maximum(point, aabb_min), aabb_max)

    def h_values_state(self, pos):
        """
        Return array of h = (distance_to_obs - (obstacle_radius + safety_margin))
        Positive = safe, negative = too close / collision.
        """
        vals = []
        threshold = self.safety_margin
        for obs in self.obstacles:
            if obs["type"] == "cuboid":
                d = self.point_to_aabb_distance(pos, obs["pos"], obs["size"])
            elif obs["type"] == "cylinder":
                center = obs["pos"]
                radius = float(obs["size"][0])
                height = float(obs["size"][2])
                d_xy = np.linalg.norm(pos[:2] - center[:2]) - radius
                half_h = height/2.0
                dz = 0.0
                if pos[2] < center[2] - half_h:
                    dz = (center[2] - half_h) - pos[2]
                elif pos[2] > center[2] + half_h:
                    dz = pos[2] - (center[2] + half_h)
                d = float(np.linalg.norm([max(d_xy, 0.0), dz]))
            else:
                d = float(np.linalg.norm(pos - obs["pos"]))
            vals.append(d - threshold)
        return np.array(vals, dtype=np.float32)

    def h_min_and_index(self, pos):
        hvals = self.h_values_state(pos)
        idx = int(np.argmin(hvals))
        return float(hvals[idx]), idx

    def dh_dv_coeff(self, pos, obs_idx):
        """
        Return gradient dh/dpos (shape (3,)). For CBF that needs dh/dv, multiply by dpos/dv (~dt).
        This implementation returns zero vector if index invalid.
        """
        if len(self.obstacles) == 0 or obs_idx < 0 or obs_idx >= len(self.obstacles):
            return np.zeros(3, dtype=np.float32)

        obs = self.obstacles[obs_idx]
        if obs["type"] == "cuboid":
            closest = self.closest_point_on_aabb(pos, obs["pos"], obs["size"])
            diff = pos - closest
            dist = np.linalg.norm(diff)
            if dist <= 1e-8:
                # point is on/in box -> gradient undefined; return outward normal approx (0)
                return np.random.uniform(-0.1,0.1,3).astype(np.float32)
            grad = diff / dist  # derivative of distance wrt pos
            # gradient of h = d - threshold -> same as grad
            return grad.astype(np.float32)
        elif obs["type"] == "cylinder":
            center = obs["pos"]
            radius = float(obs["size"][0])
            height = float(obs["size"][2])
            # compute components
            vec_xy = pos[:2] - center[:2]
            dist_xy = np.linalg.norm(vec_xy)
            half_h = height / 2.0
            # z component outside?
            if pos[2] < center[2] - half_h:
                dz = (center[2] - half_h) - pos[2]
                # gradient is [0,0,-1] for z part plus xy if outside radius
                if dist_xy > 1e-8:
                    # both xy and z outside -> grad = [vec_xy/dist_xy, -dz/|v|] normalized
                    # approximate: combine components
                    g_xy = vec_xy / dist_xy
                    g = np.array([g_xy[0], g_xy[1], -1.0], dtype=np.float32)
                    g_norm = np.linalg.norm(g)
                    if g_norm > 1e-8:
                        return (g / g_norm).astype(np.float32)
                    else:
                        return np.array([0,0,-1], dtype=np.float32)
                else:
                    return np.array([0.0, 0.0, -1.0], dtype=np.float32)
            elif pos[2] > center[2] + half_h:
                # above top
                if dist_xy > 1e-8:
                    g_xy = vec_xy / dist_xy
                    g = np.array([g_xy[0], g_xy[1], 1.0], dtype=np.float32)
                    g_norm = np.linalg.norm(g)
                    if g_norm > 1e-8:
                        return (g / g_norm).astype(np.float32)
                    else:
                        return np.array([0,0,1], dtype=np.float32)
                else:
                    return np.array([0.0, 0.0, 1.0], dtype=np.float32)
            else:
                # within z band: gradient only in xy (radial)
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

    def _min_dist_to_obstacles(self):
        """
        Return:
            min_dist: float
            closest_obs: dict
            closest_point: np.array of shape (3,)
        """
        if len(self.obstacles) == 0:
            return float("inf"), None, None

        p = self.pos

        best_dist = float("inf")
        best_obs = None
        best_point = None

        for obs in self.obstacles:

            # ====== CUBOID ======
            if obs["type"] == "cuboid":
                d = self.point_to_aabb_distance(p, obs["pos"], obs["size"])
                cp = self.closest_point_on_aabb(p, obs["pos"], obs["size"])

            # ====== CYLINDER (直立, 沿 z 軸) ======
            elif obs["type"] == "cylinder":
                center = obs["pos"]
                radius = float(obs["size"][0])
                height = float(obs["size"][2])
                half_h = height / 2.0

                # 最近點 XY（投影到圓上）
                v = p[:2] - center[:2]
                dist_xy = np.linalg.norm(v)

                if dist_xy > radius:
                    # 拉到圓周 boundary
                    cp_xy = center[:2] + v / dist_xy * radius
                else:
                    # 在圓柱內部 XY：最近點是自己
                    cp_xy = p[:2]

                # Z 高度 clamp
                z_min = center[2] - half_h
                z_max = center[2] + half_h
                cp_z = np.clip(p[2], z_min, z_max)

                cp = np.array([cp_xy[0], cp_xy[1], cp_z], dtype=np.float32)

                # 距離
                d = float(np.linalg.norm(p - cp))

            # ====== SPHERE or POINT ======
            else:
                cp = obs["pos"]
                d = float(np.linalg.norm(p - cp))

            # 比較最小距離
            if d < best_dist:
                best_dist = d
                best_obs = obs
                best_point = cp

        return best_dist, best_obs, best_point-p


    # ---------------- Rendering ----------------
    def render(self):
        if not self.render_mode:
            return

        # --- create L-shape subplots if missing ---
        if not hasattr(self, "ax_xy"):
            plt.close(self.fig)
            self.fig = plt.figure(figsize=(12, 8))
            gs = gridspec.GridSpec(2, 2, figure=self.fig, width_ratios=[2,1], height_ratios=[1,1], wspace=0.3, hspace=0.3)

            self.ax_xy = self.fig.add_subplot(gs[0,0])  # 左上
            self.ax_yz = self.fig.add_subplot(gs[0,1])  # 右上
            self.ax_xz = self.fig.add_subplot(gs[1,0])  # 左下

            plt.ion()
            plt.show()

        ax_xy = self.ax_xy
        ax_xz = self.ax_xz
        ax_yz = self.ax_yz

        ax_xy.clear(); ax_xz.clear(); ax_yz.clear()

        # ---------- (A) 顯示房間 ----------
        for i in range(self.num_rooms):
            ax_xy.add_patch(Rectangle(
                (i*self.room_size, 0),
                self.room_size, self.room_size,
                fill=False, edgecolor='gray'
            ))

        # ---------- (C) 障礙物 ----------
        for obs in self.obstacles:
            x, y, z = obs["pos"]
            if obs["type"] == "cuboid":
                sx, sy, sz = obs["size"]

                # XY
                ax_xy.add_patch(Rectangle(
                    (x - sx/2, y - sy/2), sx, sy,
                    color='red', alpha=0.4))

                # XZ
                ax_xz.add_patch(Rectangle(
                    (x - sx/2, z - sz/2), sx, sz,
                    color='red', alpha=0.3))

                # YZ
                ax_yz.add_patch(Rectangle(
                    (y - sy/2, z - sz/2), sy, sz,
                    color='red', alpha=0.3))

            else:  # cylinder
                radius, _, height = obs["size"]

                # XY
                ax_xy.add_patch(Circle((x, y), radius, color='red', alpha=0.4))

                # XZ
                ax_xz.add_patch(Rectangle(
                    (x - radius, z - height/2), radius*2, height,
                    color='red', alpha=0.3))

                # YZ
                ax_yz.add_patch(Rectangle(
                    (y - radius, z - height/2), radius*2, height,
                    color='red', alpha=0.3))

        # ---------- (D) reference line ----------
        sx, sy = self.start_pos[:2]
        gx, gy = self.goal[:2]
        ax_xy.plot([sx, gx], [sy, gy], '--k', alpha=0.5)

        # # ---------- (E) Trajectory ----------
        hist = np.array(self._history)
        if hist.shape[0] > 0:
            ax_xy.plot(hist[:,0], hist[:,1], '-b')
            ax_xz.plot(hist[:,0], hist[:,2], '-b')
            ax_yz.plot(hist[:,1], hist[:,2], '-b')

        # ---------- (F) UAV + Goal ----------
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

        # ---------- (G) Set limits ----------
        ax_xy.set_xlim(0, self.num_rooms*self.room_size)
        ax_xy.set_ylim(0, self.room_size)
        ax_xy.set_title(f"XY (t={self.t})")

        ax_xz.set_xlim(0, self.num_rooms*self.room_size)
        ax_xz.set_ylim(0, self.space_limit)
        ax_xz.set_title("XZ")

        ax_yz.set_xlim(0, self.room_size)
        ax_yz.set_ylim(0, self.space_limit)
        ax_yz.set_title("YZ")

        plt.pause(0.001)

