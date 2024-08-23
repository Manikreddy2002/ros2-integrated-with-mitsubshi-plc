import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PLCSubscriber(Node):
    def __init__(self):
        super().__init__('PLCSubscriber')
        self.subscription = self.create_subscription(
            String,
            'PLC',
            self.listener_callback,
            10
        )
        self.get_logger().info('PLCSubscriber node has been started.')

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if "error" in data:
                self.get_logger().error(f'Received error: {data["error"]}')
            else:
                self.get_logger().info(f'Received PLC data: {data}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON data received')

def main(args=None):
    rclpy.init(args=args)
    plc_subscriber = PLCSubscriber()
    rclpy.spin(plc_subscriber)
    plc_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
