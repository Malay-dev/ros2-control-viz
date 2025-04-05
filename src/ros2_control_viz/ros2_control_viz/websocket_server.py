import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from ros2_control_viz_interfaces.msg import GraphUpdate

class GraphSubscriber(Node):
    def __init__(self):
        super().__init__('graph_subscriber')
        
        # Store the current graph state for new clients
        self.nodes = {}  # Dictionary to store node metadata
        self.edges = {}  # Dictionary to store edge data
        
        self.subscription = self.create_subscription(
            GraphUpdate, 
            'graph_update',
            self.listener_callback, 
            10
        )
        self.ws_clients = set()
        self.get_logger().info('Graph subscriber initialized and waiting for connections')

    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections from the Next.js client."""
        self.ws_clients.add(websocket)
        client_info = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self.get_logger().info(f"New WebSocket client connected: {client_info}")

        # Send the current graph state to new clients
        if self.edges:
            initial_state = {
                "type": "full_graph",
                "nodes": self.nodes,
                "edges": list(self.edges.values())
            }
            await websocket.send(json.dumps(initial_state))
            self.get_logger().info(f"Sent initial graph state to {client_info}")

        try:
            # Listen for client messages (commands, interactions, etc.)
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.handle_client_message(websocket, data)
                except json.JSONDecodeError:
                    self.get_logger().warning(f"Received invalid JSON from client: {message}")
        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().info(f"WebSocket client disconnected: {client_info} (code={e.code}, reason={e.reason})")
        finally:
            self.ws_clients.remove(websocket)

    async def handle_client_message(self, websocket, data):
        """Handle messages from clients such as layout changes or interaction events"""
        if "type" not in data:
            self.get_logger().warning("Received message without type field")
            return
            
        if data["type"] == "get_graph":
            # Client requesting the full graph
            full_graph = {
                "type": "full_graph",
                "nodes": self.nodes,
                "edges": list(self.edges.values())
            }
            await websocket.send(json.dumps(full_graph))
            
        elif data["type"] == "node_position":
            # Client sending node position updates
            # Broadcast to other clients to keep visualizations synchronized
            await self.broadcast(json.dumps(data), exclude=websocket)
            
        elif data["type"] == "clear_graph":
            # Client requesting to clear the graph
            self.nodes = {}
            self.edges = {}
            clear_message = {"type": "clear_graph"}
            await self.broadcast(json.dumps(clear_message))
            
        else:
            self.get_logger().info(f"Received client message: {data}")

    def listener_callback(self, msg):
        """Process ROS 2 messages and update graph state"""
        # Generate unique edge ID
        edge_id = f"{msg.start}_{msg.end}_{msg.command_interface}_{msg.state_interface}"
        
        # Parse metadata
        metadata = msg.metadata
        try:
            metadata_dict = json.loads(metadata)
        except json.JSONDecodeError:
            metadata_dict = {"raw": metadata}
        
        # Update node information
        if msg.start not in self.nodes:
            self.nodes[msg.start] = {"id": msg.start, "metadata": {}}
        
        if msg.end not in self.nodes:
            self.nodes[msg.end] = {"id": msg.end, "metadata": {}}
            
        # Update edge information
        edge_data = {
            "id": edge_id,
            "source": msg.start,
            "target": msg.end,
            "command_interface": msg.command_interface,
            "state_interface": msg.state_interface,
            "is_hardware": msg.is_hardware,
            "metadata": metadata_dict
        }
        self.edges[edge_id] = edge_data
        
        # Create WebSocket message
        graph_update = {
            "type": "edge_update",
            "edge": edge_data
        }
        
        json_data = json.dumps(graph_update)
        self.get_logger().info(f"Broadcasting graph update: {msg.start} -> {msg.end}")

        # Send to all clients
        asyncio.create_task(self.broadcast(json_data))

    async def broadcast(self, message, exclude=None):
        """Send data to all connected WebSocket clients except the excluded one."""
        disconnected = []
        
        for client in self.ws_clients:
            if exclude and client == exclude:
                continue
                
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.append(client)
                
        # Remove disconnected clients
        for client in disconnected:
            self.ws_clients.remove(client)
            self.get_logger().info("Removed disconnected WebSocket client")

async def main_async():
    """Run ROS and WebSockets concurrently."""
    rclpy.init()
    graph_subscriber = GraphSubscriber()

    # Start WebSocket server
    ws_server = await websockets.serve(
        graph_subscriber.websocket_handler, 
        "0.0.0.0",  # Listen on all interfaces
        8765        # Port can be changed as needed
    )
    graph_subscriber.get_logger().info("WebSocket server started on ws://0.0.0.0:8765")

    # Run ROS and WebSocket server together
    ros_task = asyncio.create_task(run_ros(graph_subscriber))
    
    try:
        await asyncio.gather(ws_server.wait_closed(), ros_task)
    except KeyboardInterrupt:
        graph_subscriber.get_logger().info("Shutting down server...")
    finally:
        rclpy.shutdown()

async def run_ros(node):
    """Run ROS in an async-friendly way."""
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.01)  # Yield control to asyncio but more frequently
    except asyncio.CancelledError:
        node.get_logger().info("ROS processing task cancelled")

def main():
    """Start the event loop."""
    asyncio.run(main_async())

if __name__ == "__main__":
    main()