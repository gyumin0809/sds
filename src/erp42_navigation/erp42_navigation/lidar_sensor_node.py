def main(args=None):
    rclpy.init(args=args)
    node = LidarSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
