import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Twist
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import os
import time

class UAVEnv(gym.Env, Node):
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, render_mode=False):
        Node.__init__(self, 'tello_env')
        gym.Env.__init__(self)

        self.render_mode = render_mode
        self.dt = 0.1

        # Action = UAV velocity (x,y,z)
        self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)
        # State = UAV position
        self.observation_space = spaces.Box(low=np.array([0,0,0], dtype=np.float32),
                                            high=np.array([20,10,3], dtype=np.float32))

        self.pos = np.array([0.0, 0.0, 1.0])
        self.obstacles = []

        # ROS2 clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity...')

        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delete_entity...')

        # UAV info
        self.uav_name = "tello1"
        self.urdf_path = "/home/wenjia/CBFRL/gazebo_env/src/tello-ros2-gazebo/tello_ros/tello_description/urdf/tello.xml"

        # Velocity publisher
        self.pub_twist = self.create_publisher(Twist, f"/{self.uav_name}/cmd_vel", 10)
        self.twist = Twist()

    # -------------------------------------------------------------
    # Spawn UAV
    # -------------------------------------------------------------
    def spawn_uav(self, suffix="1"):
        if not os.path.exists(self.urdf_path):
            self.get_logger().error(f"URDF not found: {self.urdf_path}")
            return

        req = SpawnEntity.Request()
        req.name = f"{self.uav_name}_{suffix}"

        # 讀 URDF，替換 ${suffix} 為合法字元
        suffix_str = str(suffix)  # 保證是字串
        # 只允許字母、數字、底線
        suffix_str = "".join(c if c.isalnum() or c=='_' else '_' for c in suffix_str)

        with open(self.urdf_path, "r") as f:
            urdf_text = f.read()

        # 替換 URDF 中的 ${suffix} 與 ${topic_ns}
        urdf_text = urdf_text.replace("${suffix}", suffix_str)
        urdf_text = urdf_text.replace("${topic_ns}", f"tello{suffix_str}")

        req.xml = urdf_text


        # 隨機初始位置
        pose = Pose()
        pose.position.x = np.random.uniform(0.5, 19.5)
        pose.position.y = np.random.uniform(0.5, 9.5)
        pose.position.z = np.random.uniform(1.0, 2.0)
        req.initial_pose = pose

        self.spawn_client.call_async(req)
        self.get_logger().info(f"Spawned UAV: {req.name} at ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})")

    # -------------------------------------------------------------
    # Spawn obstacle
    # -------------------------------------------------------------
    def spawn_obstacle(self, name, x, y, z):
        sx, sy, sz = np.random.uniform(0.3, 1.0, size=3)
        req = SpawnEntity.Request()
        req.name = name
        req.xml = f"""
        <sdf version="1.6">
          <model name="{name}">
            <static>true</static>
            <link name="link">
              <collision name="collision">
                <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
              </collision>
              <visual name="visual">
                <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
                <material><ambient>1 0 0 1</ambient></material>
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

        self.spawn_client.call_async(req)
        return {"name": name, "x": x, "y": y, "z": z, "sx": sx, "sy": sy, "sz": sz}

    # -------------------------------------------------------------
    # Collision detection
    # -------------------------------------------------------------
    def check_collision(self, pos):
        x, y, z = pos
        if x < 0 or x > 20 or y < 0 or y > 10 or z < 0 or z > 3:
            return True
        for obs in self.obstacles:
            if (abs(x - obs["x"]) < obs["sx"]/2 and
                abs(y - obs["y"]) < obs["sy"]/2 and
                abs(z - obs["z"]) < obs["sz"]/2):
                return True
        return False

    # -------------------------------------------------------------
    # Gym step
    # -------------------------------------------------------------
    def step(self, action):
        self.twist.linear.x = float(action[0])
        self.twist.linear.y = float(action[1])
        self.twist.linear.z = float(action[2])
        self.pub_twist.publish(self.twist)

        # Internal simulation update
        self.pos += np.array(action) * self.dt

        crashed = self.check_collision(self.pos)
        reward = -np.linalg.norm(action)
        if crashed:
            reward -= 50.0
        done = crashed

        return self.pos.copy(), reward, done, False, {"crashed": crashed}

    # -------------------------------------------------------------
    # Gym reset
    # -------------------------------------------------------------
    def reset(self):
        # Delete previous obstacles
        for obs in self.obstacles:
            req = DeleteEntity.Request()
            req.name = obs["name"]
            self.delete_client.call_async(req)
        self.obstacles = []

        # Spawn new obstacles
        num_obs = np.random.randint(3, 6)
        for i in range(num_obs):
            x = np.random.uniform(0, 20)
            y = np.random.uniform(0, 10)
            z = np.random.uniform(0, 3)
            name = f"obs_{int(time.time()*1000)%100000}_{i}"
            self.obstacles.append(self.spawn_obstacle(name, x, y, z))

        self.pos = np.array([0.0, 0.0, 1.0])
        return self.pos.copy(), {}
