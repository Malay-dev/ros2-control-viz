class TurtleConnector:
    """
    Connector class that handles the translation between turtle messages
    and the graph structure needed for visualization.
    """
    def __init__(self):
        # Define the nodes in our graph
        self.nodes = {
            'teleop': {
                'id': 'teleop',
                'metadata': {
                    "type": "controller", 
                    "description": "Keyboard teleop controller",
                    "status": "active"
                }
            },
            'turtlesim': {
                'id': 'turtlesim',
                'metadata': {
                    "type": "simulator", 
                    "description": "Turtle simulator node",
                    "model": "turtle1"
                }
            },
            'cmd_vel': {
                'id': 'cmd_vel',
                'metadata': {
                    "type": "topic", 
                    "message_type": "geometry_msgs/Twist",
                    "path": "/turtle1/cmd_vel"
                }
            },
            'pose': {
                'id': 'pose',
                'metadata': {
                    "type": "topic", 
                    "message_type": "turtlesim/Pose",
                    "path": "/turtle1/pose"
                }
            }
        }
        
        # Keep track of latest velocities and pose
        self.latest_linear_velocity = 0.0
        self.latest_angular_velocity = 0.0
        self.latest_pose = None

    def process_teleop_command(self, twist_msg, timestamp):
        """Process incoming teleop commands and return edge data"""
        self.latest_linear_velocity = twist_msg.linear.x
        self.latest_angular_velocity = twist_msg.angular.z
        
        # Create edge data for teleop to cmd_vel
        edge_data = {
            'start': 'teleop',
            'end': 'cmd_vel',
            'command_interface': True,
            'state_interface': False,
            'is_hardware': False,
            'metadata': {
                "linear_x": twist_msg.linear.x,
                "angular_z": twist_msg.angular.z,
                "timestamp": timestamp
            }
        }
        
        return edge_data
    
    def process_pose_update(self, pose_msg, timestamp):
        """Process incoming pose updates and return edge data"""
        self.latest_pose = pose_msg
        
        # Create edge data for turtlesim to pose
        edge_data = {
            'start': 'turtlesim',
            'end': 'pose',
            'command_interface': False,
            'state_interface': True,
            'is_hardware': False,
            'metadata': {
                "x": round(pose_msg.x, 2),
                "y": round(pose_msg.y, 2),
                "theta": round(pose_msg.theta, 2),
                "timestamp": timestamp
            }
        }
        
        return edge_data
    
    def get_command_to_turtle_edge(self, timestamp):
        """Get the edge data from cmd_vel to turtlesim"""
        edge_data = {
            'start': 'cmd_vel',
            'end': 'turtlesim',
            'command_interface': True,
            'state_interface': False,
            'is_hardware': True,
            'metadata': {
                "latest_command": {
                    "linear_x": self.latest_linear_velocity,
                    "angular_z": self.latest_angular_velocity
                },
                "status": "active" if abs(self.latest_linear_velocity) > 0.01 or 
                                    abs(self.latest_angular_velocity) > 0.01 else "idle",
                "timestamp": timestamp
            }
        }
        
        return edge_data
        
    def get_all_nodes(self):
        """Return all nodes in the graph for complete visualization"""
        return self.nodes