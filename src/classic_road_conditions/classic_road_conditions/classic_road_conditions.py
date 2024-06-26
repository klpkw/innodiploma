import rclpy
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaTrafficLightStatusList
from carla_msgs.srv import GetTrafficLightStatus

class ClassicRoadConditions(Node):
    def __init__(self):
        super().__init__('classic_road_conditions')
        self.control_publisher = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)
        self.traffic_light_status_subscriber = self.create_subscription(
            CarlaTrafficLightStatusList, '/carla/traffic_lights/status', self.traffic_light_callback, 10)
        self.traffic_light_client = self.create_client(GetTrafficLightStatus, '/carla/traffic_lights/get_status')
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_speed = 0.5
        self.steer_angle = 0.0

    def traffic_light_callback(self, msg):
        for light in msg.traffic_lights:
            if light.state == 1:  # Red light
                self.current_speed = 0.0
            elif light.state == 2:  # Yellow light
                self.current_speed = 0.2
            else:  # Green light
                self.current_speed = 0.5

    def timer_callback(self):
        msg = CarlaEgoVehicleControl()
        msg.throttle = self.current_speed
        msg.steer = self.steer_angle
        self.control_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClassicRoadConditions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
