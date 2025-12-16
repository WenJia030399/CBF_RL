# env_gazebo_cbf_with_goal.py
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from scipy.optimize import minimize
import os, time
from tf_transformations import euler_from_quaternion

class UAVEnvCBF(gym.Env, Node):
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, render_mode=False, gamma=15.0): # ⚠️ 修正: 增大 gamma 提高鲁棒性
        Node.__init__(self, 'tello_env')
        gym.Env.__init__(self)
        self.render_mode = render_mode
        self.dt = 0.05
        self.gamma = gamma
        self.safety_margin = 0.3  # ⚠️ 修正: 增大安全裕度 (例如 0.5 -> 1.0)

        # Action = UAV velocity (x,y,z) in m/s
        self.action_space = spaces.Box(
            low=np.array([-2.0, -2.0, -2.0, -1.0]),
            high=np.array([ 2.0,  2.0,  2.0,  1.0]),
            dtype=np.float32
        )

        # Observation = UAV pos + goal pos
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(6,),
            dtype=np.float32
        )


        # state
        self.pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.yaw = 0.0
        self.vel = np.zeros(3, dtype=np.float32)
        self.goal = np.array([10.0,5.0,1.0], dtype=np.float32)
        self.start_pos = self.pos.copy()
        self.prev_dist = np.linalg.norm(self.pos - self.goal)
        self.t = 0
        self.max_steps = 2500
        self.odom_ready = False
        
        # action bounds used by projection
        self.lb = np.array([-2.0, -2.0, -2.0], dtype=np.float32)
        self.ub = np.array([ 2.0,  2.0,  2.0], dtype=np.float32)


        self.obstacles = []

        # Name / namespace handling
        self.uav_name = "tello1"
        self.uav_suffix = "1"
        self.uav_model_name = f"{self.uav_name}_{self.uav_suffix}"
        self.uav_ns = self.uav_name

        self.urdf_path = "/home/wenjia/CBFRL/gazebo_env/src/tello-ros2-gazebo/tello_ros/tello_description/urdf/tello.xml"

        # ROS2 clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity...')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delete_entity...')

        # Velocity publisher (use namespace)
        self.pub_twist = self.create_publisher(Twist, f"/{self.uav_ns}/cmd_vel", 10)
        self.twist = Twist()

        # Subscribe /odom to update UAV pos (use namespace)
        self.sub_odom = self.create_subscription(Odometry, f"/{self.uav_ns}/odom", self.odom_callback, 1)

        self.goal_model_name = "goal_marker"

    def spawn_goal_visual(self, name, x, y, z, r=0.2, wait=True, timeout=10.0):
            """Spawns a visual-only sphere marker in Gazebo."""
            req = SpawnEntity.Request()
            req.name = name
            
            # 🚨 關鍵: SDF 只有 <visual> 或 <collision> 設為 <empty/>
            req.xml = f"""
            <sdf version="1.7">
                <model name="{name}">
                    <static>true</static>
                    <link name="link">
                        <visual name="visual">
                            <geometry><sphere><radius>{r}</radius></sphere></geometry>
                            <material><ambient>1 0 0 0.8</ambient><diffuse>1 0 0 0.8</diffuse></material>
                        </visual>
                        <collision name="collision">
                            <geometry><sphere><radius>0.0001</radius></sphere></geometry>
                        </collision>
                    </link>
                </model>
            </sdf>
            """
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            req.initial_pose = pose

            fut = self.spawn_client.call_async(req)
            if wait:
                start = time.time()
                while rclpy.ok() and (time.time() - start) < timeout:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    if fut.done():
                        break
                if not fut.done():
                    self.get_logger().warn(f"Spawn visual goal {name} timed out")

            self.get_logger().info(f"Spawned visual goal at ({x:.2f},{y:.2f},{z:.2f})")
            return True

    # ------------------------- Odometry -------------------------
    def odom_callback(self, msg: Odometry):
        # --- position ---
        self.pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ], dtype=np.float32)

        # --- orientation (quaternion -> yaw) ---
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]

        _, _, self.yaw = euler_from_quaternion(quat)

        self.odom_ready = True

    def wait_for_odom(self, attempts=3, timeout_per_attempt=0.01):
        """Try to spin until we have a fresh odom; return True if odom was received."""
        # 由於 step 中會 spin_once，這裡只需檢查 odom_ready 即可
        if self.odom_ready:
            return True
        # 在 step 中已經執行 spin_once，這裡不再重複
        return False

    # ------------------------- Spawn UAV -------------------------
    def spawn_uav(self, suffix=None, wait=True, timeout=5.0):
        if suffix is None:
            suffix = self.uav_suffix
        model_name = f"{self.uav_name}_{suffix}"
        if not os.path.exists(self.urdf_path):
            self.get_logger().error(f"URDF not found: {self.urdf_path}")
            return False
        req = SpawnEntity.Request()
        req.name = model_name
        with open(self.urdf_path, "r") as f:
            urdf_text = f.read()
        urdf_text = urdf_text.replace("${suffix}", str(suffix))
        urdf_text = urdf_text.replace("${topic_ns}", f"{self.uav_name}")
        req.xml = urdf_text

        pose = Pose()
        pose.position.x = np.random.uniform(1.0, 19.0)
        pose.position.y = np.random.uniform(1.0, 9.0)
        if self.start_pos[0] != 0:
            pose.position.x = float(self.start_pos[0])
            pose.position.y = float(self.start_pos[1])
        pose.position.z = 0.15 # 修正: 設置安全起飛高度
        req.initial_pose = pose

        fut = self.spawn_client.call_async(req)
        if wait:
            start = time.time()
            while rclpy.ok() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.01)
                if fut.done():
                    break
            if not fut.done():
                self.get_logger().error(f"Spawn {model_name} did not complete within timeout")
                return False

        self.get_logger().info(f"Spawned UAV: {model_name} at ({pose.position.x:.2f},{pose.position.y:.2f},{pose.position.z:.2f})")
        self.uav_model_name = model_name
        return True

    # ------------------------- Spawn obstacle -------------------------
    def spawn_obstacle(self, name, x, y, z, size = None,wait=True, timeout=3.0):
        # create a cuboid obstacle by default; can be changed to cylinder later
        if size == None: 
            sx, sy, sz = np.random.uniform(0.5, 2.0, size=3)
        else:
            sx= size[0]
            sy= size[1]
            sz= size[2]
        c1, c2, c3 = np.random.uniform(0.0, 1.0, size=3)
        c4 = 1.0
        transparency = 0.4
        if name == "w_roof":
            transparency = 1
        req = SpawnEntity.Request()
        req.name = name
        req.xml = f"""
        <sdf version="1.7">
          <model name="{name}">
            <static>true</static>
            <link name="link">
              <collision name="collision">
                <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
              </collision>
              <visual name="visual">
                <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
                <material><ambient>{c1} {c2} {c3} {c4}</ambient></material>
              </visual>
            </link>
          </model>
        </sdf>
        """
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        req.initial_pose = pose

        fut = self.spawn_client.call_async(req)
        if wait:
            start = time.time()
            while rclpy.ok() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.01)
                if fut.done():
                    break
            if not fut.done():
                self.get_logger().warn(f"Spawn obstacle {name} timed out")

        # Return obs description with explicit type and size
        return {"name": name, "type": "cuboid", "pos": np.array([x,y,z], dtype=np.float32), "size": np.array([sx,sy,sz], dtype=np.float32), "sx": sx, "sy": sy, "sz": sz}
    
    # ------------------------- CBF helpers (修正 CBF 邏輯與冗餘清理) -------------------------
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
        Return array of h = (distance_to_obs - safety_margin)
        Positive = safe, negative = too close / collision.
        """
        vals = []
        threshold = self.safety_margin
        for obs in self.obstacles:
            if obs.get("type", "cuboid") == "cuboid":
                d = self.point_to_aabb_distance(pos, obs["pos"], obs["size"])
            elif obs.get("type") == "cylinder":
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
            # print(f"Obstacle {obs['name']}: distance={d:.3f}, h={d - threshold:.3f}")
        return np.array(vals, dtype=np.float32)

    def h_min_and_index(self, pos):
        hvals = self.h_values_state(pos)
        if hvals.size == 0:
            return float("inf"), -1
        idx = int(np.argmin(hvals))
        return float(hvals[idx]), idx
    
    def dh_dv_coeff(self, pos, obs_idx):
        if len(self.obstacles) == 0 or obs_idx < 0 or obs_idx >= len(self.obstacles):
            return np.zeros(3, dtype=np.float32)

        obs = self.obstacles[obs_idx]
        if obs.get("type", "cuboid") == "cuboid":
            center = np.array(obs["pos"], dtype=np.float64)
            size = np.array(obs["size"], dtype=np.float64)
            closest = self.closest_point_on_aabb(pos, center, size)
            diff = (pos - closest).astype(np.float64)
            dist = np.linalg.norm(diff)
            if dist <= 1e-8:
                # point is inside or exactly on surface -> choose outward normal toward the nearest face
                half = 0.5 * size
                rel = pos - center  # relative coordinates
                # distances to faces along each axis
                d_pos = half - rel  # distance to +face along axis
                d_neg = half + rel  # distance to -face along axis
                # pick axis with minimum absolute distance to a face
                axis = int(np.argmin([abs(d_pos[0]), abs(d_pos[1]), abs(d_pos[2])]))
                # determine sign: if nearer to +face -> normal is +axis else -axis
                if abs(d_pos[axis]) < abs(d_neg[axis]):
                    normal = np.zeros(3); normal[axis] = 1.0
                else:
                    normal = np.zeros(3); normal[axis] = -1.0
                return normal.astype(np.float32)
            grad = (diff / dist).astype(np.float32)
            return grad
        elif obs.get("type") == "cylinder":
            center = obs["pos"]
            radius = float(obs["size"][0])
            height = float(obs["size"][2])
            vec_xy = pos[:2] - center[:2]
            dist_xy = np.linalg.norm(vec_xy)
            half_h = height / 2.0
            if pos[2] < center[2] - half_h:
                if dist_xy > 1e-8:
                    g_xy = vec_xy / dist_xy
                    g = np.array([g_xy[0], g_xy[1], -1.0], dtype=np.float32)
                    g_norm = np.linalg.norm(g)
                    if g_norm > 1e-8:
                        return (g / g_norm).astype(np.float32)
                    else:
                        return np.array([0.,0.,-1.], dtype=np.float32)
                else:
                    return np.array([0.,0.,-1.], dtype=np.float32)
            elif pos[2] > center[2] + half_h:
                if dist_xy > 1e-8:
                    g_xy = vec_xy / dist_xy
                    g = np.array([g_xy[0], g_xy[1], 1.0], dtype=np.float32)
                    g_norm = np.linalg.norm(g)
                    if g_norm > 1e-8:
                        return (g / g_norm).astype(np.float32)
                    else:
                        return np.array([0.,0.,1.], dtype=np.float32)
                else:
                    return np.array([0.,0.,1.], dtype=np.float32)
            else:
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

    def _solve_projection(self, u_rl, pos):
        """Project u_rl into safe set using single-most-critical-obstacle QP (with fallback)."""
        # ensure bounds exist
        lb = self.lb
        ub = self.ub

        if minimize is None:
            return np.clip(u_rl, lb, ub)

        h_min, idx = self.h_min_and_index(pos)
        if idx < 0 or np.isinf(h_min):
            return np.clip(u_rl, lb, ub)

        a = self.dh_dv_coeff(pos, idx).astype(np.float64)  # shape (3,)
        h_i = float(h_min)
        
        # 🚨 CBF 關鍵修正: 假設系統為一階積分器 dot(x) = u (Lg=I, Lf=0) 以提高魯棒性
        Lf = 0.0 
        
        # CBF 約束: a·u >= -Lf - gamma * h_i
        b = -Lf - self.gamma * h_i

        # if already satisfies
        if float(np.dot(a, u_rl)) >= b - 1e-9:
            return np.clip(u_rl, lb, ub)

        # Quadratic objective: min 0.5||u-u_rl||^2
        def obj(u):
            return 0.5 * np.sum((u - u_rl)**2)
        def jac(u):
            return (u - u_rl).astype(np.float64)

        cons = ({ 'type': 'ineq',
                  'fun': lambda u, a=a, b=b: float(np.dot(a, u) - b),
                  'jac': lambda u, a=a: a.astype(np.float64)
                },)

        bounds = [(float(lb[i]), float(ub[i])) for i in range(3)]
        x0 = np.clip(u_rl, lb, ub).astype(np.float64)

        try:
            res = minimize(obj, x0, jac=jac, bounds=bounds, constraints=cons,
                           method='SLSQP', options={'ftol':1e-6, 'maxiter':50, 'disp': False})
            if res.success:
                return np.clip(res.x.astype(np.float32), lb, ub)
        except Exception as e:
            self.get_logger().debug(f"_solve_projection minimize failed: {e}")

        # fallback analytic projection onto half-space a·u >= b
        a_norm2 = np.dot(a, a)
        if a_norm2 <= 1e-9:
            return np.clip(u_rl, lb, ub)
        tau = max(0.0, (b - np.dot(a, u_rl)) / a_norm2)
        u_proj = u_rl + tau * a
        return np.clip(u_proj, lb, ub).astype(np.float32)

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

    # ------------------------- Collision Checker -------------------------
    def check_collision(self, pos):
        """
        Return True if UAV is colliding with ANY obstacle (AABB / cylinder / sphere).
        """
        # 1. World boundary check (硬編碼邊界)
        x, y, z = pos
        # 考慮到牆壁厚度 0.2，實際房間範圍為 [0, 20] x [0, 10] x [0, 3]
        if x < 0.0 or x > 20.0 or y < 0.0 or y > 10.0 or z < 0.0 or z > 3.2: # 稍微放寬 Z 上限
            return True

        # 2. Check real obstacles (與 CBF 邏輯相同)
        for obs in self.obstacles:
            otype = obs.get("type", "cuboid")

            if otype == "cuboid":
                d = self.point_to_aabb_distance(
                        pos,
                        np.array(obs["pos"]),
                        np.array(obs["size"])
                )
                if d <= 0.0:
                    return True

            elif otype == "cylinder":
                center = np.array(obs["pos"])
                r = obs["size"][0]
                h = obs["size"][2] / 2.0

                d_xy = np.linalg.norm(pos[:2] - center[:2])
                if d_xy <= r and abs(pos[2] - center[2]) <= h:
                    return True

            else:  # sphere fallback
                d = np.linalg.norm(pos - np.array(obs["pos"]))
                if d <= obs["size"][0]:
                    return True
        return False


    # ------------------------- Gym step -------------------------
    def step(self, action):
        yaw = action[-1]
        action = action[:3]
        min_obstacle_dist,_,obstacle_vector = self._min_dist_to_obstacles()
        u_rl = np.array(action, dtype=np.float32)
        u_safe = u_rl.copy()
        if min_obstacle_dist < 2.0:
            u_safe = self._solve_projection(u_rl, self.pos)
        self.vel = u_safe.copy()

        # 1. Publish to Gazeboㄑ
        self.twist.linear.x = float(u_safe[0])
        self.twist.linear.y = float(u_safe[1])
        self.twist.linear.z = float(u_safe[2])

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = float(yaw)  # yaw rate

        self.pub_twist.publish(self.twist)

        
        # 2. Spin once to allow ROS2/Gazebo to update physics and Odom
        rclpy.spin_once(self, timeout_sec=self.dt)

        # 3. Check for Odom update; if not received, use internal integrator as fallback
        got_odom = self.odom_ready
        self.odom_ready = False # Reset flag for next step
        if not got_odom:
            self.pos = self.pos + self.vel * self.dt # Fallback integration

        # collision check on updated pos
        crashed = self.check_collision(self.pos)

        # compute distances & reward
        dist_to_goal = np.linalg.norm(self.pos - self.goal)
        reward = 0.0
        progress = self.prev_dist - dist_to_goal
        reward += 5.0 * progress
        reward += -0.2 * dist_to_goal
        reward += -0.4 * abs(self.pos[2] - self.goal[2])
        reward += -0.01 * np.linalg.norm(u_safe)
        reward += -0.2 * np.linalg.norm(u_safe - u_rl)

        # # line-following reward (xy plane)
        p_xy = self.pos[:2]
        p0 = self.start_pos[:2]
        g_xy = self.goal[:2]
        d = g_xy - p0
        d_norm2 = np.dot(d, d) + 1e-9
        t_proj = np.dot(p_xy - p0, d) / d_norm2
        t_proj = np.clip(t_proj, 0.0, 1.0)
        closest = p0 + t_proj * d
        line_error = np.linalg.norm(p_xy - closest)
        # reward += -0.5 * line_error

        if dist_to_goal < 3.0:
            reward += 3.0 * (1 - np.tanh(3 * dist_to_goal))
        
       

        done = False
        info = {"min_dist_to_obstacle": float(min_obstacle_dist),
                "collision": crashed,
                "line_error": float(line_error)}
        
        yaw_e = self.yaw-self.goal_yaw
        yaw_e = np.arctan2(np.sin(yaw_e), np.cos(yaw_e))

        if crashed:
            reward -= 200.0
            done = True
            info["collision"] = True
            self.get_logger().info("Crash!")
        elif dist_to_goal < 0.1 and abs(yaw_e)<0.3:
            reward += 500.0
            done = True
            info["success"] = True
            self.get_logger().info("Success!")
        # elif self.t >= self.max_steps:
        #     done = True
        #     info["timeout"] = True
        #     self.get_logger().info("Timeout!")

        self.prev_dist = dist_to_goal
        # self.t += 1
        info["reward"] = float(reward)


        d_pos = self.pos - self.goal
        dist = np.linalg.norm(d_pos)

        obs = np.array([
            *d_pos,
            dist,
            self.yaw,
            yaw_e
        ], dtype=np.float32)
        # obs = np.concatenate([self.pos, u_safe, obstacle_vector, self.goal]).astype(np.float32)
        return obs.copy(), float(reward), bool(done), False, info
    
    def delete_models(self):
        """
        刪除障礙物與目標模型，確保不漏刪。
        """
        # 所有要刪除的模型名稱
        model_names = [obs["name"] for obs in self.obstacles]
        model_names.append(self.goal_model_name)

        for name in model_names:
            for attempt in range(3):  # 最多重試 3 次
                try:
                    req = DeleteEntity.Request()
                    req.name = name
                    fut = self.delete_client.call_async(req)
                    
                    # 等待 service 完成 (1 秒 timeout)
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
                    
                    # 檢查是否成功
                    fut.result()  # 若刪除失敗會丟 Exception
                    # 刪除成功，跳出重試迴圈
                    break
                except Exception as e:
                    print(f"[Warning] Delete '{name}' failed on attempt {attempt+1}: {e}")
                    time.sleep(0.05)  # 輕微延遲再重試
            else:
                # 3 次重試都失敗
                print(f"[Error] Failed to delete '{name}' after 3 attempts")

    # ------------------------- Gym reset -------------------------
    def reset(self, *, seed=None, options=None):
        self.odom_ready = False
        if hasattr(self, 'spawned') and self.spawned:
            try:
                req = DeleteEntity.Request()
                req.name = self.uav_model_name
                future = self.delete_client.call_async(req)
                start = time.time()
                while rclpy.ok() and (time.time() - start) < 3.0:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    if future.done():
                        break
            except Exception as e:
                self.get_logger().warn(f"Failed to delete old UAV: {e}")
            self.spawned = False

        self.delete_models()

        self.obstacles = []

        # spawn new random obstacles
        num_obs = np.random.randint(3,5)
        for i in range(num_obs):
            x = np.random.uniform(1,19) # 避免在牆壁厚度內生成
            y = np.random.uniform(1,9)
            z = np.random.uniform(0.5,2.5)
            name = f"obs_{int(time.time()*1000)%100000}_{i}"
            obs = self.spawn_obstacle(name, x, y, z)
            self.obstacles.append(obs)
        rclpy.spin_once(self, timeout_sec=0.05)

        wall = [
            ["w_left", 0.1, 5.0, 1.5, [0.2,10.0,3.0]],
            ["w_right", 19.9, 5.0, 1.5, [0.2,10.0,3.0]],
            ["w_bottom", 10.0, 0.1, 1.5, [20.0,0.2,3.0]],
            ["w_top", 10.0, 9.9, 1.5, [20.0,0.2,3.0]],
            ["mid_lower", 10.0, 1.5, 1.5, [0.2,np.random.uniform(0,2),1.0]],
            ["mid_upper", 10.0, 8.5, 1.5, [0.2,np.random.uniform(0,2),1.0]],
            # ["w_roof", 10.0, 5.0, 3.1, [20.0,10.0,0.2]]
        ]
        for w in wall:
            obs = self.spawn_obstacle(w[0], w[1], w[2], w[3], size=w[4])
            self.obstacles.append(obs)

        rclpy.spin_once(self, timeout_sec=0.05)

        # spawn new UAV (blocking wait)
        ok = self.spawn_uav(suffix=self.uav_suffix, wait=True, timeout=5.0)
        self.spawned = ok

        # generate random goal
        self.goal = np.array([
            np.random.uniform(1,19),
            np.random.uniform(1,9),
            np.random.uniform(0.5,2.5)
        ], dtype=np.float32)

        self.goal_yaw = float(np.random.uniform(-np.pi, np.pi))
        
        # self.spawn_goal_visual(
        #     name=self.goal_model_name,
        #     x=self.goal[0],
        #     y=self.goal[1],
        #     z=self.goal[2],
        #     r=0.2 # 視覺標記半徑
        # )

        # attempt takeoff if service exists
        try:
            from tello_msgs.srv import TelloAction
            client = self.create_client(TelloAction, f'/{self.uav_ns}/tello_action')
            start = time.time()
            while not client.wait_for_service(timeout_sec=1.0) and (time.time() - start) < 5.0:
                self.get_logger().info("Waiting for takeoff service...")
            if client.wait_for_service(timeout_sec=0.5):
                req = TelloAction.Request()
                req.cmd = "takeoff"
                future = client.call_async(req)
                start = time.time()
                while rclpy.ok() and (time.time() - start) < 3.0:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    if future.done():
                        break
            else:
                self.get_logger().warn("TelloAction service not available; skipping takeoff")
        except Exception as e:
            self.get_logger().debug(f"Takeoff failed (or tello_msgs missing): {e}")

        self.vel = np.zeros(3, dtype=np.float32)
        self.start_pos = self.pos.copy()
        self.prev_dist = np.linalg.norm(self.pos - self.goal)
        self.t = 0

        #new obs
        d_pos = self.pos - self.goal
        dist = np.linalg.norm(d_pos)
        yaw_e = self.yaw - self.goal_yaw
        obs = np.array([
            *d_pos,
            dist,
            self.yaw,
            yaw_e
        ], dtype=np.float32)
        # obs = np.concatenate([self.pos, np.zeros(3), np.zeros(3),self.goal]).astype(np.float32)
        return obs.copy(), {}