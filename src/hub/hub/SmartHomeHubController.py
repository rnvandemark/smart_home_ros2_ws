from time import sleep, localtime, strftime
from math import cos as cosine, pi as PI
from datetime import datetime, timedelta
from threading import Thread, Lock

import matplotlib.pyplot as plt

from rclpy import init, spin, shutdown

from smart_home_msgs.msg import ModeChange, CountdownState

from SmartHomeHubNode import SmartHomeHubNode
#from AudioHandler import AudioHandler

#
# Constants
#

BATCHING_WAIT_TIME_S            = 0.1
CLOCK_UPDATE_WAIT_TIME_S        = 0.5
TRAFFIC_LIGHT_FLASH_WAIT_TIME_S = 0.5
TRAFFIC_LIGHT_FLASH_COUNT       = 3
WAVE_PERIOD_DEFAULT_S           = 10.0
WAVE_UPDATE_WAIT_TIME_S         = 0.03333
WAVE_NEGLIGIBLE_THRESHOLD       = 0.01
AUDIO_PROCESSING_WAIT_TIME_S    = 0.1
AUDIO_PROCESSING_FRAME_COUNT    = 32
_2PI                            = 2 * PI

MONTH_ABBREVIATIONS = {
	1: "Jan",  2: "Feb",  3: "Mar",  4: "Apr",
	5: "May",  6: "Jun",  7: "Jul",  8: "Aug",
	9: "Sep", 10: "Oct", 11: "Nov", 12: "Dec"
}

DAYS = {
	0: "Monday", 1: "Tuesday",  2: "Wednesday", 3: "Thursday",
	4: "Friday", 5: "Saturday", 6: "Sunday"
}

#
# Global functions
#

## Given a date, create a user-friendly string for it.
#
#  @param weekday The integer corresponding to the day of the week (M to Su).
#  @param month The integer corresponding to the month of the year.
#  @param day The integer corresponding to the day of the month.
#  @return A formatted string for the date and time.
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

#
# Class definition
#

## A controller for the smart home hub node, and a utility for the GUI as well.
class SmartHomeHubController():
	
	## The constructor. Creates references for all and initializes most of the prallel threads, as well
	#  as binds or passes on references to function callbacks.
	#
	#  @param self The object pointer.
	#  @param args The program arguments, which are passed on to the ROS client library's initialization.
	#  @param clock_update_handler The function callback for the thread updating the GUI's clock.
	#  @param mode_type_change_handler The function callback for the node to inform the GUI of a state change.
	#  @param traffic_light_change_handler The function callback for the node to tell what state the traffic
	#  light is in.
	def __init__(self, args, clock_update_handler, mode_type_change_handler, traffic_light_change_handler):
		self.keep_peripheral_threads_alive = True
		
		self.spin_thread                   = Thread(target=self._do_spin)
		self.clock_update_thread           = Thread(target=self._do_clock_updates)
		self.intensity_check_thread        = Thread(target=self._do_intensity_batching)
		self.local_audio_processing_thread = Thread(target=self._do_local_audio_processing)
		self.wave_update_check_thread      = Thread(target=self._do_wave_update_batching)
		self.wave_processing_thread        = Thread(target=self._do_wave_processing)
		self.traffic_light_flash_thread    = None
		
		#self.audio_handler = AudioHandler()
		
		self.clock_update_handler = clock_update_handler
		
		self.intensity_change_mutex  = Lock()
		self.queued_intensity_change = None
		
		self.wave_update_change_mutex  = Lock()
		self.queued_wave_update_change = None
		self.wave_period               = WAVE_PERIOD_DEFAULT_S
		self.wave_position             = 0.0
		
		self.wave_position_increment = None
		self._set_wave_position_increment()
		
		init(args=args)
		self.hub_node = SmartHomeHubNode(mode_type_change_handler, traffic_light_change_handler)
	
	## The routine for the spin thread to perform.
	#
	#  @param self The object pointer.
	def _do_spin(self):
		self.hub_node.get_logger().info("Starting spin routine.")
		spin(self.hub_node)
		self.hub_node.get_logger().info("Exiting spin routine.")
	
	## The routine for the clock udpate thread to perform.
	#
	#  @param self The object pointer.
	def _do_clock_updates(self):
		while self.keep_peripheral_threads_alive:
			local_time = localtime()
			self.clock_update_handler(
				get_formatted_day_str(local_time.tm_wday, local_time.tm_mon, local_time.tm_mday),
				strftime("%I:%M:%S ", local_time) + ("AM" if local_time.tm_hour < 12 else "PM")
			)
			sleep(CLOCK_UPDATE_WAIT_TIME_S)
	
	## The routine for the intensity queuing thread to perform.
	#
	#  @param self The object pointer.
	def _do_intensity_batching(self):
		while self.keep_peripheral_threads_alive:
			if self.hub_node.current_mode == ModeChange.INDIVIDUAL_CONTROL:
				self.intensity_change_mutex.acquire()
				try:
					if self.queued_intensity_change != None:
						self.hub_node.send_intensity_change(self.queued_intensity_change)
						self.queued_intensity_change = None
				finally:
					self.intensity_change_mutex.release()
			
			sleep(BATCHING_WAIT_TIME_S)
	
	## The routine to perform handling and processing for the audio coming out of the hub's speakers.
	#
	#  @param self The object pointer.
	def _do_local_audio_processing(self):
		while self.keep_peripheral_threads_alive:
			if self.hub_node.current_mode == ModeChange.FOLLOW_COMPUTER_SOUND:
				sleep(AUDIO_PROCESSING_WAIT_TIME_S)
			else:
				sleep(AUDIO_PROCESSING_WAIT_TIME_S)
	
	## The routine for the wave revolution period change queuing thread to perform.
	#
	#  @param self The object pointer.
	def _do_wave_update_batching(self):
		while self.keep_peripheral_threads_alive:
			if self.hub_node.current_mode == ModeChange.WAVE:
				self.wave_update_change_mutex.acquire()
				try:
					if self.queued_wave_update_change != None:
						self.wave_period = self.queued_wave_update_change
						self._set_wave_position_increment()
						self.queued_wave_update_change = None
				finally:
					self.wave_update_change_mutex.release()
			
			sleep(BATCHING_WAIT_TIME_S)
	
	## The routine for the wave revolution period change queuing thread to perform.
	#
	#  @param self The object pointer.
	def _do_wave_processing(self):
		while self.keep_peripheral_threads_alive:
			if self.hub_node.current_mode == ModeChange.WAVE:
				self.wave_update_change_mutex.acquire()
				try:
					self.wave_position = self.wave_position + self.wave_position_increment
					while self.wave_position >= _2PI:
						self.wave_position = self.wave_position - _2PI
					
					id_intensity_pairs = []
					for p_id, p_location in self.hub_node.participant_locations:
						intensity = cosine(p_location - self.wave_position)
						if intensity < WAVE_NEGLIGIBLE_THRESHOLD:
							intensity = 0.0
						
						id_intensity_pairs.append((p_id, intensity))
					
					self.hub_node.send_wave_update(id_intensity_pairs)
				finally:
					self.wave_update_change_mutex.release()
			
			sleep(WAVE_UPDATE_WAIT_TIME_S)
	
	## The routine for the clock update thread to perform.
	#
	#  @param self The object pointer.
	def _do_traffic_light_monitoring(self):
		for i in range(TRAFFIC_LIGHT_FLASH_COUNT):
			if not self.keep_peripheral_threads_alive:
				break
			
			self.hub_node.send_countdown_state(CountdownState.TO_ALL)
			sleep(TRAFFIC_LIGHT_FLASH_WAIT_TIME_S)
			self.hub_node.send_countdown_state(CountdownState.TO_NONE)
			sleep(TRAFFIC_LIGHT_FLASH_WAIT_TIME_S)
		
		obj_time, green_time, yellow_time, red_time = self.hub_node.active_mode_sequence_characteristics[1:5]
		last_countdown_state_sent = CountdownState.TO_NONE
		is_past_obj_time = False
		while (self.keep_peripheral_threads_alive
				and (self.hub_node.active_mode_sequence_characteristics[0] == ModeChange.MORNING_COUNTDOWN)):
			local_time = self.get_local_time()
			
			if local_time == obj_time:
				is_past_obj_time = True
				break
			elif local_time == red_time:
				if last_countdown_state_sent != CountdownState.TO_RED:
					self.hub_node.send_countdown_state(CountdownState.TO_RED)
					last_countdown_state_sent = CountdownState.TO_RED
			elif local_time == yellow_time:
				if ((last_countdown_state_sent != CountdownState.TO_YELLOW)
						and (last_countdown_state_sent != CountdownState.TO_RED)):
					self.hub_node.send_countdown_state(CountdownState.TO_YELLOW)
					last_countdown_state_sent = CountdownState.TO_YELLOW
			elif local_time == green_time:
				if ((last_countdown_state_sent != CountdownState.TO_GREEN)
						and (last_countdown_state_sent != CountdownState.TO_YELLOW)
						and (last_countdown_state_sent != CountdownState.TO_RED)):
					self.hub_node.send_countdown_state(CountdownState.TO_GREEN)
					last_countdown_state_sent = CountdownState.TO_GREEN
			
			sleep(1)
		
		if is_past_obj_time:
			while (self.keep_peripheral_threads_alive
					and (self.hub_node.active_mode_sequence_characteristics[0] == ModeChange.MORNING_COUNTDOWN)):
				self.hub_node.send_countdown_state(CountdownState.TO_RED)
				sleep(TRAFFIC_LIGHT_FLASH_WAIT_TIME_S)
				
				if not (self.keep_peripheral_threads_alive
					and (self.hub_node.active_mode_sequence_characteristics[0] == ModeChange.MORNING_COUNTDOWN)):
					break
				
				self.hub_node.send_countdown_state(CountdownState.TO_NONE)
				sleep(TRAFFIC_LIGHT_FLASH_WAIT_TIME_S)
	
	## The routine for the traffic light state update thread to perform.
	#
	#  @param self The object pointer.
	def _trigger_traffic_light_monitoring(self):
		self.traffic_light_flash_thread = Thread(target=self._do_traffic_light_monitoring)
		self.traffic_light_flash_thread.start()
	
	## A helper function to set the wave mode's position incrementer based on the period and "framerate".
	#
	#  @param self The object pointer.
	def _set_wave_position_increment(self):
		self.wave_position_increment = (1.0 / self.wave_period) * WAVE_UPDATE_WAIT_TIME_S * _2PI
	
	## Starts the threads that are to always be active.
	#
	#  @param self The object pointer.
	def start(self):
		self.spin_thread.start()
		self.local_audio_processing_thread.start()
		self.wave_processing_thread.start()
		self.wave_update_check_thread.start()
		self.intensity_check_thread.start()
		self.clock_update_thread.start()
		
		#self.audio_handler.start()
	
	## A blocking function that unblocks when the application is shutting down.
	#
	#  @param self The object pointer.
	def block_until_shutdown(self):
		#self.audio_handler.join()
		self.clock_update_thread.join()
		self.intensity_check_thread.join()
		self.wave_update_check_thread.join()
		self.wave_processing_thread.join()
		self.local_audio_processing_thread.join()
		
		if self.traffic_light_flash_thread:
			self.traffic_light_flash_thread.join()
		
		self.spin_thread.join()
	
	## Called when a shutdown flag should be set, so the application can be cleanly terminated.
	#
	#  @param self The object pointer.
	def stop(self):
		self.keep_peripheral_threads_alive = False
		#self.audio_handler.stop()
		
		self.hub_node.get_logger().info("Doing destruction.")
		self.hub_node.destroy_node()
		shutdown()
	
	## Formats the current local time into hours, minutes, and AM/PM.
	#
	#  @param self The object pointer.
	#  @param local_time The time to format. If unprovided or otherwise null, the current time is fetched.
	#  @return A tuple describing the hour (1-12), minute(0-59), and "AM"/"PM" for the current time.
	def get_local_time(self, local_time=None):
		if not local_time:
			local_time = localtime()
		
		hour = local_time.tm_hour % 12
		if hour == 0:
			hour = 12
		
		return (hour, local_time.tm_min, "AM" if local_time.tm_hour < 12 else "PM")
	
	## Attempts to take an intensity change that was requested to be sent and attempts to prioritize it. This
	#  will lock the mutex and then set the intensity value, so as to guarantee the updating thread isn't interrupted.
	#
	#  @param self The object pointer.
	#  @param intensity The intensity to be queued, normalized from [0.0, 1.0].
	def request_intensity_change(self, intensity):
		self.intensity_change_mutex.acquire()
		try:
			self.queued_intensity_change = intensity
		finally:
			self.intensity_change_mutex.release()
	
	## Attempts to take a wave period change that was requested to be sent and attempts to prioritize it. This
	#  will lock the mutex and then set the wave period value, so as to guarantee the updating thread isn't interrupted.
	#
	#  @param self The object pointer.
	#  @param wave_update The wave period to be queued, from 1 (once a min) to 60 (once every 60 mins / once an hour).
	def request_wave_update_change(self, wave_update):
		self.wave_update_change_mutex.acquire()
		try:
			self.queued_wave_update_change = wave_update
		finally:
			self.wave_update_change_mutex.release()
	
	## Tells the node to inform the ROS network of a mode change.
	#
	#  @param self The object pointer.
	#  @param new_mode_type The mode type ROS constant that is not active.
	#  @param call_handler Whether or not to call the GUI's bounded handler.
	def send_mode_type(self, new_mode_type, call_handler):
		previous_mode_type = self.hub_node.current_mode
		
		self.hub_node.active_mode_sequence_characteristics = (-1,)
		self.hub_node.send_mode_type(new_mode_type, call_handler)
		
		#self.audio_handler.set_collection_status(new_mode_type == ModeChange.FOLLOW_COMPUTER_SOUND)
	
	## A helper function to set the mode to FULL_ON and call the handler.
	#
	#  @param self The object pointer.
	def set_full_on_mode_type(self):
		self.send_mode_type(ModeChange.FULL_ON, True)
	
	## A helper function to set the mode to FULL_OFF and call the handler.
	#
	#  @param self The object pointer.
	def set_full_off_mode_type(self):
		self.send_mode_type(ModeChange.FULL_OFF, True)
	
	## Set the active characteristics for the newly set mode type and perform any setup for it.
	#
	#  @param self The object pointer.
	#  @param mode_type The ROS constant for the mode type.
	#  @param characteristics Any values/parameters that are important to know for the new mode type.
	def set_active_sequence_for_mode_type(self, mode_type, *characteristics):
		if self.traffic_light_flash_thread:
			self.hub_node.active_mode_sequence_characteristics = (-1,)
			self.traffic_light_flash_thread.join()
		
		self.hub_node.active_mode_sequence_characteristics = (mode_type, *characteristics)
		
		if mode_type == ModeChange.MORNING_COUNTDOWN:
			self.hub_node.send_countdown_state(CountdownState.CONFIRMATION)
			self._trigger_traffic_light_monitoring()
	
	## Calculate the number of minutes until a provided time is reached.
	#
	#  @param self The object pointer.
	#  @param obj_hour The hour (1-12) to meet.
	#  @param obj_minute The minute (0-59) to meet.
	#  @param obj_am_pm Either "AM" or "PM".
	#  @return A tuple containing the current time used to perform the calculations and the resulting
	#  number of whole minutes.
	def get_mins_until(self, obj_hour, obj_minute, obj_am_pm):
		curr_time = localtime()
		
		if obj_hour == 12:
			obj_hour = 0
		
		if obj_am_pm == "PM":
			obj_hour = obj_hour + 12
		
		curr_time_str = str(curr_time.tm_hour) + ":" + str(curr_time.tm_min)
		obj_time_str  = str(obj_hour) + ":" + str(obj_minute)
		fmt_tim_str   = "%H:%M"
		
		time_delta = datetime.strptime(obj_time_str, fmt_tim_str) - datetime.strptime(curr_time_str, fmt_tim_str)
		if time_delta.days < 0:
			time_delta = time_delta + timedelta(days=1)
		
		if time_delta.days != 0:
			raise ValueError("Invalid timedelta [" + str(time_delta) + "]")
		
		return (curr_time, int(time_delta.total_seconds() / 60))