import rclpy
from rclpy.node import Node
from ros2_control_viz_interfaces.msg import GraphUpdate
import json
import random

class GraphPublisher(Node):
    def __init__(self):
        super().__init__('graph_publisher')
        self.publisher_ = self.create_publisher(GraphUpdate, 'graph_update', 10)
        
        # Sample graph data with more comprehensive network
        self.graph_data = [
            {'start': 'A', 'end': 'B', 'command_interface': True, 'state_interface': False, 'is_hardware': False, 
             'metadata': json.dumps({"type": "controller", "priority": 1, "rate": "100Hz"})},
            {'start': 'B', 'end': 'C', 'command_interface': False, 'state_interface': True, 'is_hardware': False, 
             'metadata': json.dumps({"type": "state_handler", "priority": 2, "status": "active"})},
            {'start': 'A', 'end': 'D', 'command_interface': True, 'state_interface': False, 'is_hardware': True, 
             'metadata': json.dumps({"type": "hardware_interface", "device": "motor1", "manufacturer": "Dynamixel"})},
            {'start': 'D', 'end': 'E', 'command_interface': False, 'state_interface': True, 'is_hardware': True, 
             'metadata': json.dumps({"type": "sensor", "device": "encoder1", "resolution": "1024 ticks"})},
            {'start': 'C', 'end': 'F', 'command_interface': True, 'state_interface': False, 'is_hardware': False, 
             'metadata': json.dumps({"type": "output", "target": "display", "format": "JSON"})},
            {'start': 'F', 'end': 'G', 'command_interface': False, 'state_interface': True, 'is_hardware': True, 
             'metadata': json.dumps({"type": "actuator", "model": "XYZ123", "status": "ready"})},
            {'start': 'B', 'end': 'H', 'command_interface': True, 'state_interface': False, 'is_hardware': False, 
             'metadata': json.dumps({"type": "processor", "algorithm": "PID", "parameters": {"P": 0.5, "I": 0.1, "D": 0.01}})}
        ]
        
        # Options for demo mode
        self.publish_all = True  # Set to True to publish all edges in sequence
        self.random_mode = False  # Set to True to publish random edges
        
        self.current_index = 0
        self.timer = self.create_timer(1.0, self.publish_graph_update)

    def publish_graph_update(self):
        """Publish graph updates either sequentially or randomly"""
        if self.publish_all and self.current_index < len(self.graph_data):
            # Sequential publishing mode
            edge_data = self.graph_data[self.current_index]
            self.current_index += 1
            
            # If we've published all edges, loop back to the beginning
            if self.current_index >= len(self.graph_data):
                self.get_logger().info('Published all edges, resetting sequence')
                self.current_index = 0
        elif self.random_mode:
            # Random publishing mode for more dynamic demonstration
            edge_data = random.choice(self.graph_data)
        else:
            # Default single edge example from original code
            edge_data = {
                'start': 'A', 
                'end': 'B', 
                'command_interface': True, 
                'state_interface': False, 
                'is_hardware': False, 
                'metadata': json.dumps({"type": "example", "description": "Simple connection"})
            }
        
        # Create and publish the message
        msg = GraphUpdate()
        msg.start = edge_data['start']
        msg.end = edge_data['end']
        msg.command_interface = edge_data['command_interface']
        msg.state_interface = edge_data['state_interface']
        msg.is_hardware = edge_data['is_hardware']
        msg.metadata = edge_data['metadata']
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.start} -> {msg.end} (Command: {msg.command_interface}, State: {msg.state_interface}, Hardware: {msg.is_hardware})')

def main(args=None):
    rclpy.init(args=args)
    node = GraphPublisher()
    rclpy.spin(node)
    node.destroy_node()  # Added proper cleanup
    rclpy.shutdown()

if __name__ == '__main__':
    main()