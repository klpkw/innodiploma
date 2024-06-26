import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl

class ParkingManeuver(Node):
    def __init__(self):
        super().__init__('parking_maneuver')
        self.control_publisher = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.step = 0

    def timer_callback(self):
        msg = CarlaEgoVehicleControl()
        if self.step < 20:  # Move forward
            msg.throttle = 0.3
            msg.steer = 0.0
        elif self.step < 40:  # Steer right
            msg.throttle = 0.1
            msg.steer = 0.5
        elif self.step < 60:  # Steer left and reverse
            msg.throttle = 0.1
            msg.steer = -0.5
            msg.reverse = True
        else:  # Straight reverse
            msg.throttle = 0.3
            msg.steer = 0.0
            msg.reverse = True
        self.control_publisher.publish(msg)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = ParkingManeuver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
