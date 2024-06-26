import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl
import math

class SnakePath(Node):
    def __init__(self):
        super().__init__('snake_path')
        self.control_publisher = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time = 0.0

    def timer_callback(self):
        msg = CarlaEgoVehicleControl()
        msg.throttle = 0.4
        msg.steer = 0.5 * math.sin(self.time)
        self.control_publisher.publish(msg)
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = SnakePath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
