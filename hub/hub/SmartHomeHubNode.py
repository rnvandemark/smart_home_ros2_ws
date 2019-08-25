from rclpy.node import Node

from std_msgs.msg import UInt8, Float32
from smart_home_msgs.msg import ModeChange, ModeChangeRequest, DeviceActivationChange, CountdownState

class SmartHomeHubNode(Node):
	
	def __init__(self, mode_type_change_handler, traffic_light_change_handler):
		super(SmartHomeHubNode, self).__init__('smart_home_hub')
		
		#
		# Publishers
		#
		
		self.mode_change_pub = self.create_publisher(
			ModeChange,
			"/smart_home/mode_change_chatter",
			1
		)
		
		self.intensity_change_pub = self.create_publisher(
			Float32,
			"/smart_home/intensity_change_chatter",
			1
		)
		
		self.device_activation_change_pub = self.create_publisher(
			DeviceActivationChange,
			"/smart_home/device_activation_change_chatter",
			32
		)
		
		self.countdown_state_pub = self.create_publisher(
			CountdownState,
			"/smart_home/countdown_state_chatter",
			4
		)
		
		#
		# Subscribers
		#
		
		self.mode_change_request_sub = self.create_subscription(
			ModeChangeRequest,
			"/smart_home/mode_change_request_chatter",
			self.node_change_request_callback,
			1
		)
		
		self.countdown_state_sub = self.create_subscription(
			CountdownState,
			"/smart_home/countdown_state_chatter",
			self.countdown_state_callback,
			4
		)
		
		#
		# Other initialization
		#
		
		self.mode_type_change_handler     = mode_type_change_handler
		self.traffic_light_change_handler = traffic_light_change_handler
		
		self.current_mode = None
		self.active_mode_sequence_characteristics = (-1,)
		self.send_mode_type(ModeChange.FULL_OFF, False)
		
		self.get_logger().info("Started.")
	
	def node_change_request_callback(self, msg):
		self.send_mode_type(msg.mode_type, True)
	
	def countdown_state_callback(self, msg):
		self.traffic_light_change_handler(msg.state)
	
	def send_mode_type(self, mode_type, call_handler):
		mode_change_msg = ModeChange()
		mode_change_msg.mode_type = mode_type
		mode_change_msg.header.stamp = self.get_clock().now().to_msg()
		self.mode_change_pub.publish(mode_change_msg)
		
		self.current_mode = mode_type
		self.get_logger().info("Set mode to [{0}].".format(self.current_mode))
		
		if call_handler:
			self.mode_type_change_handler()
	
	def send_intensity_change(self, intensity):
		intensity_change_msg = Float32()
		intensity_change_msg.data = intensity
		self.intensity_change_pub.publish(intensity_change_msg)
		self.get_logger().info("Set intensity to [{0}].".format(intensity))
	
	def send_countdown_state(self, state):
		countdown_state_msg = CountdownState()
		countdown_state_msg.state = state
		self.countdown_state_pub.publish(countdown_state_msg)
		self.get_logger().info("Set countdown state to [{0}].".format(state))