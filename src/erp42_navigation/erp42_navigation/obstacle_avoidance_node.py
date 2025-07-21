#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import struct

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.get_logger().info("Obstacle Avoidance Node Started")

        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.pointcloud_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.is_avoiding_human = False
        self.human_avoid_counter = 0

    def pointcloud_callback(self, msg):
        xyz = self.convert_pointcloud2_to_numpy(msg)
        steering, velocity = self.calculate_control_commands(xyz)

        twist_msg = Twist()
        twist_msg.linear.x = float(velocity)
        # The original steering is in degrees. Twist's angular.z is typically rad/s.
        # We'll pass the degree value for the driver node to handle.
        twist_msg.angular.z = float(steering)
        self.publisher_.publish(twist_msg)

    def convert_pointcloud2_to_numpy(self, msg):
        point_step = msg.point_step
        data = msg.data
        points = []
        for i in range(0, len(data), point_step):
            # Assuming point_step is 12 for x, y, z float32
            x, y, z = struct.unpack_from('fff', data, i)
            points.append([x, y, z])
        return np.array(points, dtype=np.float32)

    def calculate_control_commands(self, xyz):
        if xyz.shape[0] == 0:
            return 0, 7  # No points, continue straight

        xyz_temp = xyz[(xyz[:, 2] > 0)]
        if xyz_temp.shape[0] == 0:
            return 0, 7

        z_data = xyz_temp[:, 2]
        x_data = xyz_temp[:, 0]
        y_data = xyz_temp[:, 1]

        # --- Object Detection ---
        human_indices = [i for i, z in enumerate(z_data) if 1.2 < z < 1.8]
        xy_human = [
            (round(x_data[i], 2), round(y_data[i], 2))
            for i in human_indices
            if -1 < x_data[i] < 1 and 0 < y_data[i] < 5
        ]

        static_indices = [i for i, value in enumerate(z_data) if 0 < value < 0.14]
        xy_static_raw = [
            (round(x_data[i], 1), round(y_data[i], 1)) for i in static_indices
        ]
        xy_static = list(
            set([p for p in xy_static_raw if -1.5 < p[0] < 1.5 and 0 < p[1] < 3.5])
        )

        x_far = [p[0] for p in xy_static if 3.0 < p[1] < 10]

        dynamic_indices = [i for i, value in enumerate(z_data) if 0.4 < value < 1]
        xy_dynamic_raw = [
            (round(x_data[i], 1), round(y_data[i], 1)) for i in dynamic_indices
        ]
        xy_dynamic = list(
            set([p for p in xy_dynamic_raw if -0.72 < p[0] < 0.72 and 0 < p[1] < 3.0])
        )

        # --- Control Logic ---
        steering = 0
        velocity = 7

        if self.is_avoiding_human:
            self.human_avoid_counter -= 1
            if self.human_avoid_counter <= 0:
                self.is_avoiding_human = False

        for (x, y) in xy_human:
            if y < 0.6 and abs(x) < 0.3:
                self.get_logger().warn("Human too close! Evasive maneuver.")
                velocity, self.is_avoiding_human, self.human_avoid_counter = 2, True, 10
                steering = 15 if x >= 0 else -15
                return steering, velocity
            elif y < 1.2 and abs(x) < 0.8:
                self.get_logger().info("Avoiding human.")
                velocity, self.is_avoiding_human, self.human_avoid_counter = 3, True, 10
                steering = int(-4 * x - 17) if x >= 0 else int(-4 * x + 17)
                return max(-15, min(15, steering)), velocity

        if len(xy_dynamic) > 0:
            self.get_logger().error("Dynamic obstacle detected! Emergency Stop!")
            return 0, 0

        if len(xy_static) > 0:
            nearest = sorted(xy_static, key=lambda p: p[1])[0]
            x_avg = nearest[0]
            self.get_logger().info(f"Avoiding static obstacle at x={x_avg}")
            velocity = 4
            steering = int(-4 * x_avg - 17) if x_avg >= 0 else int(-4 * x_avg + 17)
            steering = max(-15, min(15, steering))

        elif len(x_far) > 0:
            self.get_logger().info("Far obstacle detected, increasing speed.")
            velocity = 8

        return steering, velocity

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
