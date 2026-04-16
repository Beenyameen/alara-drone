import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import time

class ToucherNode(Node):
	def __init__(self):
		super().__init__('toucher_node')
		time.sleep(2.0)  # Wait a moment for sim_world_node to be ready before starting to subscribe.
		depth_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
		self.subscription = self.create_subscription(
			Image,
			'depth/image_raw',
			self.depth_callback,
			depth_qos
		)
		self.get_logger().info('Toucher node initialized, listening to depth/image_raw')

	def depth_callback(self, msg):
		# Just touch the topic to ensure discovery
		pass


def main(args=None):
	rclpy.init(args=args)
	node = ToucherNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()