import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaCollisionEvent

class DeerTest(Node):
    def __init__(self):
        super().__init__('deer_test')
        self.control_publisher = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        self.collision_subscriber = self.create_subscription(CarlaCollisionEvent, '/carla/ego_vehicle/collision', self.collision_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.emergency_brake = False

    def collision_callback(self, msg):
        self.emergency_brake = True

    def timer_callback(self):
        msg = CarlaEgoVehicleControl()
        if self.emergency_brake:
            msg.throttle = 0.0
            msg.brake = 1.0
        else:
            msg.throttle = 0.5
            msg.steer = 0.0
        self.control_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeerTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
