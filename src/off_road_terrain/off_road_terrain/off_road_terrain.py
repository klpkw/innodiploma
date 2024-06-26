import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl

class OffRoadTerrain(Node):
    def __init__(self):
        super().__init__('off_road_terrain')
        self.control_publisher = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.step = 0

    def timer_callback(self):
        msg = CarlaEgoVehicleControl()
        if self.step < 30:  # Move up a hill
            msg.throttle = 0.4
            msg.steer = 0.0
        elif self.step < 60:  # Navigate a bump
            msg.throttle = 0.2
            msg.steer = 0.2
        else:  # Adjust speed and trajectory on rough terrain
            msg.throttle = 0.3
            msg.steer = -0.2
        self.control_publisher.publish(msg)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = OffRoadTerrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
