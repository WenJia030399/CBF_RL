import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from env_gazebo_v1 import UAVEnv
from gazebo_msgs.srv import SpawnEntity
import time

def wait_for_spawn_service(node, timeout=30):
    """等待 /spawn_entity service 可用"""
    client = node.create_client(SpawnEntity, '/spawn_entity')
    start = time.time()
    while not client.wait_for_service(timeout_sec=1.0):
        if time.time() - start > timeout:
            raise TimeoutError("Timeout waiting for /spawn_entity service")
        node.get_logger().info("Waiting for /spawn_entity service...")
    return client

class UAVController(Node):
    def __init__(self, uav_env: UAVEnv):
        super().__init__('uav_controller')
        self.env = uav_env

        # 訂閱速度 topic
        self.sub = self.create_subscription(
            Twist,
            '/drone1/cmd_vel',  # 自訂 topic
            self.cmd_callback,
            10
        )
        self.get_logger().info("Subscribed to /uav_cmd_vel")

    def cmd_callback(self, msg: Twist):
        # 發送到 Gazebo cmd_vel
        self.env.pub_twist.publish(msg)

        # 更新 UAV 內部位置 (approximation)
        self.env.pos += [msg.linear.x, msg.linear.y, msg.linear.z] * self.env.dt

        # 碰撞檢測
        crashed = self.env.check_collision(self.env.pos)
        if crashed:
            self.get_logger().warn("UAV crashed! Resetting...")
            self.env.reset()
        else:
            self.get_logger().info(f"pos={self.env.pos}, crashed={crashed}")

def main():
    rclpy.init()

    # 初始化 UAV 環境
    env = UAVEnv(render_mode=False)

    # 等待 /spawn_entity
    wait_for_spawn_service(env)
    env.get_logger().info("/spawn_entity service ready.")

    # spawn UAV
    env.spawn_uav()
    env.reset()

    # 建立控制 node
    controller = UAVController(env)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down...")
    finally:
        controller.destroy_node()
        env.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
