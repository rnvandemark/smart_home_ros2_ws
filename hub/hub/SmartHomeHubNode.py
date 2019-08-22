import rclpy
from rclpy.node import Node

from smart_home_msgs.msg import ModeChange
from smart_home_msgs.msg import ModeChangeRequest

class SmartHomeHubNode(Node):
	
	def __init__(self):
		super(SmartHomeHubNode, self).__init__('smart_home_hub')
		
		self.mode_change_pub = self.create_publisher(
			ModeChange,
			"/smart_home_hub/mode_change_chatter",
			1
		)
		
		self.mode_change_request_sub = self.create_subscription(
			ModeChangeRequest,
			"/smart_home_hub/mode_change_request_chatter",
			self.node_change_request_callback,
			1
		)
		
		self.get_logger().info("Started node!")
	
	def node_change_request_callback(self, msg):
		mode_change_msg = ModeChange()
		mode_change_msg.mode_type = msg.mode_type
		mode_change_msg.header.stamp = self.get_clock().now().to_msg()
		self.mode_change_pub.publish(mode_change_msg)