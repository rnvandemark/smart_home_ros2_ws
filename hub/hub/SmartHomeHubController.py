from rclpy import init, spin, shutdown
from threading import Thread, Lock
from time import sleep, localtime, strftime

from smart_home_msgs.msg import ModeChange

from SmartHomeHubNode import SmartHomeHubNode

INTENSITY_CHANGE_WAIT_TIME_S = 0.2
CLOCK_UPDATE_WAIT_TIME_S     = 0.5

MONTH_ABBREVIATIONS = {
	1: "Jan",  2: "Feb",  3: "Mar",  4: "Apr",
	5: "May",  6: "Jun",  7: "Jul",  8: "Aug",
	9: "Sep", 10: "Oct", 11: "Nov", 12: "Dec"
}

DAYS = {
	0: "Monday", 1: "Tuesday",  2: "Wednesday", 3: "Thursday",
	4: "Friday", 5: "Saturday", 6: "Sunday"
}

def get_formatted_day_str(weekday, month, day):
	suffix = "th"
	if int(day / 10) != 1:
		if day % 10 == 1:
			suffix = "st"
		elif day % 10 == 2:
			suffix = "nd"
		elif day % 10 == 3:
			suffix = "rd"
	
	return "{0}, {1} {2}{3}".format(
		DAYS[weekday],
		MONTH_ABBREVIATIONS[month],
		day,
		suffix
	)

class SmartHomeHubController():
	
	def __init__(self, args, mode_type_change_handler, clock_update_handler):
		self.spin_thread = Thread(target=self._do_spin)
		
		self.keep_peripheral_threads_alive = True
		self.intensity_check_thread = Thread(target=self._do_intensity_batching)
		self.clock_update_thread = Thread(target=self._do_clock_updates)
		
		self.clock_update_handler = clock_update_handler
		
		self.intensity_change_mutex = Lock()
		self.queued_intensity_change = None
		
		init(args=args)
		self.mode_change_node = SmartHomeHubNode(mode_type_change_handler)
	
	def _do_spin(self):
		self.mode_change_node.get_logger().info("Starting spin routine.")
		spin(self.mode_change_node)
		self.mode_change_node.get_logger().info("Exiting spin routine.")
	
	def _do_intensity_batching(self):
		while self.keep_peripheral_threads_alive:
			if self.mode_change_node.current_mode == ModeChange.INDIVIDUAL_CONTROL:
				self.intensity_change_mutex.acquire()
				try:
					if self.queued_intensity_change != None:
						self.mode_change_node.send_intensity_change(self.queued_intensity_change)
						self.queued_intensity_change = None
				finally:
					self.intensity_change_mutex.release()
			
			sleep(INTENSITY_CHANGE_WAIT_TIME_S)
	
	def _do_clock_updates(self):
		while self.keep_peripheral_threads_alive:
			local_time = localtime()
			self.clock_update_handler(
				get_formatted_day_str(local_time.tm_wday, local_time.tm_mon, local_time.tm_mday),
				strftime("%H:%M:%S ", local_time) + ("AM" if local_time.tm_hour < 12 else "PM")
			)
			sleep(CLOCK_UPDATE_WAIT_TIME_S)
	
	def start(self):
		self.spin_thread.start()
		self.intensity_check_thread.start()
		self.clock_update_thread.start()
	
	def block_until_shutdown(self):
		self.clock_update_thread.join()
		self.intensity_check_thread.join()
		self.spin_thread.join()
	
	def stop(self):
		self.keep_peripheral_threads_alive = False
		
		self.mode_change_node.get_logger().info("Doing destruction.")
		self.mode_change_node.destroy_node()
		shutdown()
	
	def request_intensity_change(self, intensity):
		self.intensity_change_mutex.acquire()
		try:
			self.queued_intensity_change = intensity
		finally:
			self.intensity_change_mutex.release()