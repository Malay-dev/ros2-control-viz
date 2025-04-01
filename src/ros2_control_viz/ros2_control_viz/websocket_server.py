import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from ros2_control_viz_interfaces.msg import GraphUpdate  # Your custom message

class GraphSubscriber(Node):
    def __init__(self):
        super().__init__('graph_subscriber')
        self.subscription = self.create_subscription(
            GraphUpdate, 
            'graph_update',  # The topic publishing graph updates
            self.listener_callback, 
            10
        )
        self.ws_clients = set()  # 🔥 Renamed from self.clients to self.ws_clients

    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections from the Next.js client."""
        self.ws_clients.add(websocket)
        self.get_logger().info(f"New WebSocket client connected: {websocket.remote_address}")

        try:
            async for _ in websocket:
                pass
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f"WebSocket client disconnected: {websocket.remote_address}")
        finally:
            self.ws_clients.remove(websocket)

    def listener_callback(self, msg):
        """Convert ROS 2 message to JSON and send via WebSockets."""
        graph_data = {
            "start": msg.start,
            "end": msg.end,
            "command_interface": msg.command_interface,
            "state_interface": msg.state_interface,
            "is_hardware": msg.is_hardware,
            "metadata": msg.metadata
        }
        json_data = json.dumps(graph_data)
        self.get_logger().info(f"Publishing to Web: {json_data}")

        asyncio.create_task(self.broadcast(json_data))

    async def broadcast(self, message):
        """Send data to all connected WebSocket clients."""
        if self.ws_clients:
            await asyncio.gather(*[self.safe_send(client, message) for client in self.ws_clients])

    async def safe_send(self, client, message):
        """Safely send messages to clients and handle disconnections."""
        try:
            await client.send(message)
        except websockets.exceptions.ConnectionClosed:
            self.ws_clients.remove(client)
            self.get_logger().info("A WebSocket client disconnected.")

async def main_async():
    """Run ROS and WebSockets concurrently."""
    rclpy.init()
    graph_subscriber = GraphSubscriber()

    # Start WebSocket server
    ws_server = await websockets.serve(graph_subscriber.websocket_handler, "0.0.0.0", 8765)
    graph_subscriber.get_logger().info("WebSocket server started on ws://0.0.0.0:8765")

    # Run ROS and WebSocket server together
    ros_task = asyncio.create_task(run_ros(graph_subscriber))
    await asyncio.gather(ws_server.wait_closed(), ros_task)

async def run_ros(node):
    """Run ROS in an async-friendly way."""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0)  # Yield control to asyncio

def main():
    """Start the event loop."""
    asyncio.run(main_async())

if __name__ == "__main__":
    main()
