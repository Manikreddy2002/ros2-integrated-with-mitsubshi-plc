import time
import json
from opcua import Client
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OPCUAClientNode(Node):
    def __init__(self):
        super().__init__('OPCUAClientNode')

        # Initialize publisher for PLC data
        self.publisher = self.create_publisher(String, 'PLC', 10)

        # Replace with your OPC UA server's IP address and port
        opcua_server_ip = '192.168.0.1'
        opcua_server_port = 4001

        # Connect to OPC UA server
        self.opcua_client = Client(f"opc.tcp://{opcua_server_ip}:{opcua_server_port}/freeopcua/server/")
        self.connect_opcua_client()

        # Example: Replace with actual node IDs in your OPC UA server
        self.holding_register_node = None
        self.coil_node = None

    def connect_opcua_client(self):
        connected = False
        while not connected:
            try:
                self.opcua_client.connect()
                connected = True
                self.get_logger().info('Connected to OPC UA server')
                # Example: Replace with actual node IDs in your OPC UA server
                self.holding_register_node = self.opcua_client.get_node('ns=2;i=2')  # Example node ID
                self.coil_node = self.opcua_client.get_node('ns=2;i=3')  # Example node ID
            except Exception as e:
                self.get_logger().error(f'Connection failed: {e}')
                self.publish_error(f'Connection failed: {e}')
                time.sleep(5)  # Retry every 5 seconds

    def publish_error(self, error_message):
        msg = String()
        msg.data = json.dumps({"error": error_message})
        self.publisher.publish(msg)

    def read_holding_register(self):
        if self.holding_register_node:
            try:
                return self.holding_register_node.get_value()
            except Exception as e:
                error_message = f'Read holding register failed: {e}'
                self.get_logger().error(error_message)
                self.publish_error(error_message)
        else:
            warning_message = 'Holding register node not initialized'
            self.get_logger().warning(warning_message)
            self.publish_error(warning_message)

    def read_coil(self):
        if self.coil_node:
            try:
                return self.coil_node.get_value()
            except Exception as e:
                error_message = f'Read coil failed: {e}'
                self.get_logger().error(error_message)
                self.publish_error(error_message)
        else:
            warning_message = 'Coil node not initialized'
            self.get_logger().warning(warning_message)
            self.publish_error(warning_message)

    def write_holding_register(self, value):
        if self.holding_register_node:
            try:
                self.holding_register_node.set_value(value)
                self.get_logger().info(f'Wrote {value} to holding register')
            except Exception as e:
                error_message = f'Write holding register failed: {e}'
                self.get_logger().error(error_message)
                self.publish_error(error_message)
        else:
            warning_message = 'Holding register node not initialized'
            self.get_logger().warning(warning_message)
            self.publish_error(warning_message)

def main(args=None):
    rclpy.init(args=args)
    node = OPCUAClientNode()
    node.get_logger().info('OPCUAClientNode has been started.')
    
    # Example usage: Write to the holding register
    node.write_holding_register(12345)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down OPCUAClientNode')
    finally:
        node.opcua_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
