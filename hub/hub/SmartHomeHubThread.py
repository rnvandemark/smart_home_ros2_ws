import rclpy
from threading import Thread

from SmartHomeHubNode import SmartHomeHubNode

class SmartHomeHubThread(Thread):
	
	def __init__(self, args, parent, handler):
		super(SmartHomeHubThread, self).__init__(target=self._do_spin)
		
		rclpy.init(args=args)
		self.mode_change_node = SmartHomeHubNode(parent, handler)
	
	def _do_spin(self):
		self.mode_change_node.get_logger().info("Starting spin routine.")
		rclpy.spin(self.mode_change_node)
		self.mode_change_node.get_logger().info("Exiting spin routine.")
	
	def stop(self):
		self.mode_change_node.get_logger().info("Starting destruction.")
		self.mode_change_node.destroy_node()
		rclpy.shutdown()