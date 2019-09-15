from rclpy.node import Node
from math import pi as PI

from std_msgs.msg import Empty, UInt8, Float32
from smart_home_msgs.msg import ModeChange, ModeChangeRequest, DeviceActivationChange, CountdownState, WaveParticipantLocation, WaveUpdate

#
# Class definitions
#

## A class extending ROS2's node class, acting as the means of ROS connectivity for the smart home GUI.
class SmartHomeHubNode(Node):
	
	## The constructor. Makes the ROS connections (publishers, subscriptions, etc.) and initializes the
	#  mode type.
	#
	#  @param self The object pointer.
	#  @param mode_type_change_handler The function callback for the GUI when a request to change the
	#  mode type is sent.
	#  @param traffic_light_change_handler The function callback for the GUI when a the state of the
	#  traffic light is changed.
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
		
		self.start_wave_mode_pub = self.create_publisher(
			Empty,
			"/smart_home/start_wave_mode_chatter",
			1
		)
		
		self.intensity_change_pub = self.create_publisher(
			Float32,
			"/smart_home/intensity_change_chatter",
			1
		)
		
		self.wave_update_pub = self.create_publisher(
			WaveUpdate,
			"/smart_home/wave_update_chatter",
			128
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
		
		self.participant_location_sub = self.create_subscription(
			WaveParticipantLocation,
			"/smart_home/wave_participant_location_chatter",
			self.participant_location_callback,
			32
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
		
		self.participant_locations = None
		
		self.get_logger().info("Started.")
	
	## Repackages and sends out the requested mode type.
	#
	#  @param self The object pointer.
	#  @param msg The ROS message describing the node change request.
	def node_change_request_callback(self, msg):
		self.send_mode_type(msg.mode_type, True)
	
	## A handler for countdown state changes. The data is sent back to the GUI to set its traffic light.
	#
	#  @param self The object pointer.
	#  @param msg The ROS message describing the updated countdown state.
	def countdown_state_callback(self, msg):
		self.traffic_light_change_handler(msg.state)
	
	## A handler for a wave mode participant's response.
	#
	#  @param self The object pointer.
	#  @param msg The ROS message describing an appliance participating in the wave along with its location.
	def participant_location_callback(self, msg):
		self.traffic_light_change_handler((msg.participant_id, msg.position * PI / 180.0))
	
	## The routine to take the provided mode type ROS constant and send a new message.
	#
	#  @param self The object pointer.
	#  @param mode_type The new mode type.
	#  @param call_handler Whether or not to call the appropriate handler from the GUI.
	def send_mode_type(self, mode_type, call_handler):
		mode_change_msg = ModeChange()
		mode_change_msg.mode_type = mode_type
		mode_change_msg.header.stamp = self.get_clock().now().to_msg()
		self.mode_change_pub.publish(mode_change_msg)
		
		self.current_mode = mode_type
		self.get_logger().info("Set mode to [{0}].".format(self.current_mode))
		
		if call_handler:
			self.mode_type_change_handler()
		
		if mode_type == ModeChange.WAVE:
			self.participant_locations = []
			self.start_wave_mode_pub.publish(Empty())
		else:
			self.participant_locations = None
	
	## A helper function to package the batched intensity and publish it to the ROS network.
	#
	#  @param self The object pointer.
	#  @param id_intensity_pairs A list of tuples of size 2, containing the participant IDs and their
	#  corresponding intensities.
	def send_wave_update(self, id_intensity_pairs):
		wave_update_msg = WaveUpdate()
		
		for pair in id_intensity_pairs:
			wave_update_msg.participant_ids.append(pair[0])
			wave_update_msg.intensities.append(pair[1])
		
		self.wave_update_pub.publish(wave_update_msg)
	
	## A helper function to package the batched intensity and publish it to the ROS network.
	#
	#  @param self The object pointer.
	#  @param intensity The normalized intensity to send.
	def send_intensity_change(self, intensity):
		intensity_change_msg = Float32()
		intensity_change_msg.data = intensity
		self.intensity_change_pub.publish(intensity_change_msg)
		self.get_logger().info("Set intensity to [{0}].".format(intensity))
	
	## A helper function to take a requested state and send it out to the ROS network.
	#
	#  @param self The object pointer.
	#  @param state The requested countdown state.
	def send_countdown_state(self, state):
		countdown_state_msg = CountdownState()
		countdown_state_msg.state = state
		self.countdown_state_pub.publish(countdown_state_msg)
		self.get_logger().info("Set countdown state to [{0}].".format(state))