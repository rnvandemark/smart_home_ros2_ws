import rclpy
from threading import Thread

from SmartHomeHubNode import SmartHomeHubNode

class SmartHomeHubThread(Thread):
	
	def __init__(self, args):
		super(SmartHomeHubThread, self).__init__()
		
		rclpy.init(args=args)
		
		self.mode_change_node = SmartHomeHubNode()
	
	def stop(self):
		self.mode_change_node.destroy_node()
		rclpy.shutdown()