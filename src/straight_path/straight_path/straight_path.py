import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl

class StraightPath(Node):
    def __init__(self):
        super().__init__('straight_path')
        self.control_publisher = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = CarlaEgoVehicleControl()
        msg.throttle = 0.5
        msg.steer = 0.0
        self.control_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StraightPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
