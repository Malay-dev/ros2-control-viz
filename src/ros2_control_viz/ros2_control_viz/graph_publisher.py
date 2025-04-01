import rclpy
from rclpy.node import Node
from ros2_control_viz_interfaces.msg import GraphUpdate

class GraphPublisher(Node):
    def __init__(self):
        super().__init__('graph_publisher')
        self.publisher_ = self.create_publisher(GraphUpdate, 'graph_update', 10)
        self.timer = self.create_timer(1.0, self.publish_graph_update)

    def publish_graph_update(self):
        msg = GraphUpdate()
        msg.start = "A"
        msg.end = "B"
        msg.command_interface = True
        msg.state_interface = False
        msg.is_hardware = False
        msg.metadata = "Example Metadata"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = GraphPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
