import rclpy
from rclpy.node import Node
from ros2_control_viz_interfaces.msg import GraphUpdate
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import json

from ros2_control_viz.turtle_connector import TurtleConnector

class GraphPublisher(Node):
    """
    Publisher node that listens to turtle topics and publishes
    graph updates for visualization.
    """
    def __init__(self):
        super().__init__('turtle_graph_publisher')
        
        # Create the connector
        self.connector = TurtleConnector()
        
        # Publisher for graph visualization
        self.graph_publisher = self.create_publisher(
            GraphUpdate, 'graph_update', 10)
        
        # Subscribe to teleop cmd_vel topic
        self.teleop_subscription = self.create_subscription(
            Twist, '/turtle1/cmd_vel', self.teleop_callback, 10)
            
        # Subscribe to turtle pose
        self.pose_subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Timer to periodically update graph visualization
        self.timer = self.create_timer(0.5, self.publish_graph_updates)
        
        self.get_logger().info('Turtle graph publisher initialized')
        
    def teleop_callback(self, msg):
        """Process incoming teleop commands"""
        timestamp = self.get_clock().now().to_msg().sec
        edge_data = self.connector.process_teleop_command(msg, timestamp)
        self.publish_edge_data(edge_data)
        
    def pose_callback(self, msg):
        """Process incoming pose updates"""
        timestamp = self.get_clock().now().to_msg().sec
        edge_data = self.connector.process_pose_update(msg, timestamp)
        self.publish_edge_data(edge_data)
        
    def publish_graph_updates(self):
        """Regularly publish the complete graph structure"""
        timestamp = self.get_clock().now().to_msg().sec
        edge_data = self.connector.get_command_to_turtle_edge(timestamp)
        self.publish_edge_data(edge_data)
        
    def publish_edge_data(self, edge_data):
        """Convert edge data to ROS message and publish"""
        msg = GraphUpdate()
        msg.start = edge_data['start']
        msg.end = edge_data['end']
        msg.command_interface = edge_data['command_interface']
        msg.state_interface = edge_data['state_interface']
        msg.is_hardware = edge_data['is_hardware']
        msg.metadata = json.dumps(edge_data['metadata'])
        
        self.graph_publisher.publish(msg)
        self.get_logger().debug(f'Published edge: {msg.start} -> {msg.end}')


def main(args=None):
    rclpy.init(args=args)
    publisher = GraphPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()