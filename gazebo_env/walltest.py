#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


class WallSpawner(Node):
    def __init__(self):
        super().__init__('cbf_wall_spawner')

        # --------- Delete existing models ----------
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")
        self.get_logger().info("Waiting for /delete_entity service...")
        self.delete_client.wait_for_service()
        self.get_logger().info("/delete_entity ready.")

        # --------- Spawn new models ----------
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Waiting for /spawn_entity service...")
        self.client.wait_for_service()
        self.get_logger().info("/spawn_entity ready.")

    # ---------------- Delete model ----------------
    def delete_model(self, name):
        req = DeleteEntity.Request()
        req.name = name

        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"[DEL OK] {name}")
        else:
            self.get_logger().warn(f"[DEL FAIL] {name}")

    # ----------- Delete your walls -------------
    def delete_old_walls(self):
        walls = ["w_left", "w_right", "w_bottom", "w_top", "w_roof"]
        self.get_logger().info("Deleting old walls if exist...")
        for w in walls:
            self.delete_model(w)

    # ---------------- Spawn box -------------------
    def spawn_box(self, name, pos, size, color=[0.5, 1, 1, 1.0]):
        sx, sy, sz = size
        x, y, z = pos

        sdf = f"""
        <sdf version="1.6">
          <model name="{name}">
            <static>true</static>
            <link name="link">
              <pose>{x} {y} {z} 0 0 0</pose>

              <collision name="collision">
                <geometry>
                  <box>
                    <size>{sx} {sy} {sz}</size>
                  </box>
                </geometry>
              </collision>

              <visual name="visual">
                <geometry>
                  <box>
                    <size>{sx} {sy} {sz}</size>
                  </box>
                </geometry>
                <material>
                  <ambient>{color[0]} {color[1]} {color[2]} {color[3]}</ambient>
                  <diffuse>{color[0]} {color[1]} {color[2]} {color[3]}</diffuse>
                </material>
              </visual>

            </link>
          </model>
        </sdf>
        """

        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.robot_namespace = ""
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(f"[SPAWN OK] {name}")
        else:
            self.get_logger().error(f"[SPAWN FAIL] {name}: {future.exception()}")

    # ---------------- Spawn all CBF walls ----------------
    def spawn_all_walls(self):
        walls = [
            ("w_left",   [0.05, 2.5,0.75],  [0.2, 10.0, 3.0]),
            ("w_right",  [9.95, 2.5, 0.75], [0.2, 10.0, 3.0]),
            ("w_bottom", [5.0, 0.05, 0.75], [20.0, 0.2, 3.0]),
            ("w_top",    [5.0, 4.95, 0.75], [20.0, 0.2, 3.0]),
            ("w_roof",   [5.0, 2.5, 1.5], [20.0, 10.0, 0.2]),
        ]

        for name, pos, size in walls:
            self.spawn_box(name, pos, size)


def main():
    rclpy.init()
    node = WallSpawner()

    # Remove old walls first
    node.delete_old_walls()

    # Spawn new walls
    node.spawn_all_walls()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
