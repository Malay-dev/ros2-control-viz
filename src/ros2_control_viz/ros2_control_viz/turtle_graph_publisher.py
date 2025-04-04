import rclpy
from rclpy.node import Node
from ros2_control_viz_interfaces.msg import GraphUpdate
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import json

class TurtleConnectorPublisher(Node):
    def __init__(self):
        super().__init__('turtle_connector_publisher')
        
        # Publisher for graph visualization
        self.graph_publisher = self.create_publisher(
            GraphUpdate, 'graph_update', 10)
        
        # Subscribe to teleop cmd_vel topic
        self.teleop_subscription = self.create_subscription(
            Twist, '/turtle1/cmd_vel', self.teleop_callback, 10)
            
        # Subscribe to turtle pose
        self.pose_subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Define the nodes in our graph
        self.nodes = {
            'teleop': {
                'id': 'teleop',
                'metadata': json.dumps({
                    "type": "controller", 
                    "description": "Keyboard teleop controller",
                    "status": "active"
                })
            },
            'turtlesim': {
                'id': 'turtlesim',
                'metadata': json.dumps({
                    "type": "simulator", 
                    "description": "Turtle simulator node",
                    "model": "turtle1"
                })
            },
            'cmd_vel': {
                'id': 'cmd_vel',
                'metadata': json.dumps({
                    "type": "topic", 
                    "message_type": "geometry_msgs/Twist",
                    "path": "/turtle1/cmd_vel"
                })
            },
            'pose': {
                'id': 'pose',
                'metadata': json.dumps({
                    "type": "topic", 
                    "message_type": "turtlesim/Pose",
                    "path": "/turtle1/pose"
                })
            }
        }
        
        # Keep track of latest velocities and pose
        self.latest_linear_velocity = 0.0
        self.latest_angular_velocity = 0.0
        self.latest_pose = None
        
        # Timer to periodically update graph visualization
        self.timer = self.create_timer(0.5, self.publish_graph_updates)
        
        self.get_logger().info('Turtle connector publisher initialized')
        
    def teleop_callback(self, msg):
        """Process incoming teleop commands"""
        self.latest_linear_velocity = msg.linear.x
        self.latest_angular_velocity = msg.angular.z
        
        # Publish a command interface edge from teleop to cmd_vel
        self.publish_edge(
            'teleop', 'cmd_vel', 
            command_interface=True, 
            state_interface=False,
            is_hardware=False,
            metadata=json.dumps({
                "linear_x": msg.linear.x,
                "angular_z": msg.angular.z,
                "timestamp": self.get_clock().now().to_msg().sec
            })
        )
        
    def pose_callback(self, msg):
        """Process incoming pose updates"""
        self.latest_pose = msg
        
        # Publish a state interface edge from turtlesim to pose
        self.publish_edge(
            'turtlesim', 'pose', 
            command_interface=False, 
            state_interface=True,
            is_hardware=False,
            metadata=json.dumps({
                "x": round(msg.x, 2),
                "y": round(msg.y, 2),
                "theta": round(msg.theta, 2),
                "timestamp": self.get_clock().now().to_msg().sec
            })
        )
        
    def publish_graph_updates(self):
        """Regularly publish the complete graph structure"""
        # Command interface: cmd_vel to turtlesim
        self.publish_edge(
            'cmd_vel', 'turtlesim', 
            command_interface=True, 
            state_interface=False,
            is_hardware=True,
            metadata=json.dumps({
                "latest_command": {
                    "linear_x": self.latest_linear_velocity,
                    "angular_z": self.latest_angular_velocity
                },
                "status": "active" if abs(self.latest_linear_velocity) > 0.01 or 
                                     abs(self.latest_angular_velocity) > 0.01 else "idle"
            })
        )
        
    def publish_edge(self, start, end, command_interface, state_interface, is_hardware, metadata):
        """Helper to publish a graph edge"""
        msg = GraphUpdate()
        msg.start = start
        msg.end = end
        msg.command_interface = command_interface
        msg.state_interface = state_interface
        msg.is_hardware = is_hardware
        msg.metadata = metadata
        
        self.graph_publisher.publish(msg)
        self.get_logger().debug(f'Published edge: {start} -> {end}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleConnectorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()