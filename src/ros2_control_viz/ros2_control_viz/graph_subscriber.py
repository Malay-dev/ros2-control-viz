import rclpy
from rclpy.node import Node
from ros2_control_viz_interfaces.msg import GraphUpdate

class GraphSubscriber(Node):
    def __init__(self):
        super().__init__('graph_subscriber')
        self.subscription = self.create_subscription(
            GraphUpdate, 'graph_update', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.start} -> {msg.end} | Metadata: {msg.metadata}')

def main(args=None):
    rclpy.init(args=args)
    node = GraphSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
