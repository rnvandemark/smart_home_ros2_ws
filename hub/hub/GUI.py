from time import sleep
import tkinter
from PIL import Image, ImageTk

from ament_index_python.packages import get_package_prefix
from smart_home_msgs.msg import ModeChange, CountdownState

from SmartHomeHubController import SmartHomeHubController, WAVE_PERIOD_DEFAULT_S

#
# Constants
#

LIGHT_YELLOW_COLOR = "#F0E68C"

MODES_DICT = {
	ModeChange.FULL_OFF             : ("Full Off",           "#0F2F2F"),
	ModeChange.FULL_ON              : ("Full On",            "#3CCB2C"),
	ModeChange.INDIVIDUAL_CONTROL   : ("Individual Control", LIGHT_YELLOW_COLOR),
	ModeChange.MORNING_COUNTDOWN    : ("Morning Countdown",  LIGHT_YELLOW_COLOR),
	ModeChange.FOLLOW_COMPUTER_SOUND: ("Follow Hub",         LIGHT_YELLOW_COLOR),
	ModeChange.FOLLOW_ROOM_SOUND    : ("Follow Room",        LIGHT_YELLOW_COLOR),
	ModeChange.WAVE                 : ("Wave",               LIGHT_YELLOW_COLOR)
}

MAIN_TEXT_COLOR           = "#652828"
MAIN_COMPONENT_COLOR      = "#CDCDCD"
SECONDARY_TEXT_COLOR      = LIGHT_YELLOW_COLOR
SECONDARY_COMPONENT_COLOR = "#555555"

TEXT_INDENTATION = 15

MAIN_CANVAS_GRID_ROW_COUNT    = 11
MAIN_CANVAS_GRID_COLUMN_COUNT = 10

TRAFFIC_LIGHT_CANVAS_IMAGE_TAG = "traffic_light_canvas_image"

HUB_INSTALL_LIB_DIRECTORY = get_package_prefix("hub")

#
# Global functions
#

## Given a smart home mode, get a user-friendly name and a color code for it.
#
#  @param val The key in the dictionary.
#  @return A tuple of the name and color code.
def get_mode_characteristics(val):
	if val in MODES_DICT:
		return MODES_DICT[val]
	else:
		return ("UNDEFINED", "#FF0000")

## Access a resource in the installation library.
#
#  @param url_suffix The name of the directory and file to access.
#  @return The absolute URL of the desired resource.
def get_resource_url(url_suffix):
	return HUB_INSTALL_LIB_DIRECTORY + "/lib/hub/resources/" + url_suffix

## Given one of the states that the traffic light can be in, get that corresponding image.
#
#  @param state The state of the light. This can be a string of one of the following values:
#    "none", "green", "yellow", "red", or "all"
#  @return The desired resource as a PIL Image.
def generate_traffic_light_image_for(state):
	return Image.open(
		get_resource_url("traffic_lights/traffic_light_{0}.png".format(state))
	).resize((200, 500), Image.ANTIALIAS)

#
# Class definition
#

## The class encapsulating the display for the app's contents.
class GUI():
	
	## The constructor. Performs a lot of initialization for graphics, for all
	#  of the supported modes.
	#
	#  @param self The object pointer.
	#  @param args The program arguments, which are passed on to the ROS client
	#  library's initialization.
	def __init__(self, args):
		self.initialized = False
		
		self.hub_controller = SmartHomeHubController(
			args,
			self.handle_clock_change,
			self.handle_mode_type_change,
			self.handle_traffic_light_change
		)
		
		self.tk_root = tkinter.Tk()
		
		self.screen_width = self.tk_root.winfo_screenwidth()
		self.screen_height = self.tk_root.winfo_screenheight()
		
		original_background_image = Image.open(get_resource_url("kyoto.png"))
		original_background_image = original_background_image.resize((self.screen_width, self.screen_height), Image.ANTIALIAS)
		self.background_image = ImageTk.PhotoImage(original_background_image)
		
		self.mode_widgets = {}
		for k in MODES_DICT.keys():
			self.mode_widgets[k] = []
		
		self.main_canvas = tkinter.Canvas(
			self.tk_root,
			width=self.screen_width,
			height=self.screen_height,
			highlightthickness=0
		)
		self.main_canvas.grid(
			row=0,
			column=0,
			rowspan=MAIN_CANVAS_GRID_ROW_COUNT,
			columnspan=MAIN_CANVAS_GRID_COLUMN_COUNT
		)
		
		self.main_canvas.create_image((0, 0), image=self.background_image, anchor=tkinter.NW)
		
		#
		# The canvas text
		#
		
		self.main_canvas.create_text(
			(TEXT_INDENTATION, 0),
			text="Hey there, beautiful ;)",
			font="MSGothic 42 bold",
			fill=MAIN_TEXT_COLOR,
			anchor=tkinter.NW
		)
		
		self.current_day_time_label = self.main_canvas.create_text(
			(TEXT_INDENTATION, self.screen_height - TEXT_INDENTATION),
			font="MSGothic 16 bold",
			fill=MAIN_TEXT_COLOR,
			anchor=tkinter.SW
		)
		
		self.main_canvas.create_text(
			(self.screen_width - TEXT_INDENTATION, 0),
			text="Current Mode:",
			font="MSGothic 20 bold",
			fill=MAIN_TEXT_COLOR,
			anchor=tkinter.NE
		)
		
		self.current_mode_value_label = self.main_canvas.create_text(
			(self.screen_width - TEXT_INDENTATION, 40),
			font="MSGothic 20 bold",
			anchor=tkinter.NE
		)
		
		#
		# The four main control widgets
		# (power on/off, mode dropdown, and mode confirmation)
		#
		
		self.power_off_btn = tkinter.Button(
			self.tk_root,
			text="Turn Everything Off",
			font="MSGothic 16 bold",
			background="#CD2828",
			activebackground="#FF0000",
			command=self.hub_controller.set_full_off_mode_type
		)
		self.power_off_btn.grid(row=2, column=0, rowspan=2, columnspan=3, sticky="nsew", padx=50, pady=25)
		
		self.power_on_btn = tkinter.Button(
			self.tk_root,
			text="Turn Everything On",
			font="MSGothic 16 bold",
			background="#3CCB2C",
			activebackground="#2CF516",
			command=self.hub_controller.set_full_on_mode_type
		)
		self.power_on_btn.grid(row=4, column=0, rowspan=2, columnspan=3, sticky="nsew", padx=50, pady=25)
		
		modes_strings = [(str(k) + ": " + v[0]) for k, v in MODES_DICT.items()]
		self.mode_selection_dropdown_variable = tkinter.StringVar()
		self.mode_selection_dropdown_variable.set(modes_strings[0])
		self.mode_selection_dropdown = tkinter.OptionMenu(
			self.tk_root,
			self.mode_selection_dropdown_variable,
			*modes_strings
		)
		self.mode_selection_dropdown.config(
			font="MSGothic 16 bold",
			foreground=MAIN_TEXT_COLOR,
			background=MAIN_COMPONENT_COLOR,
			activeforeground=SECONDARY_TEXT_COLOR,
			activebackground=SECONDARY_COMPONENT_COLOR
		)
		self.mode_selection_dropdown.grid(row=6, column=0, rowspan=2, columnspan=3, sticky="nsew", padx=50, pady=25)
		
		self.confirm_option_btn = tkinter.Button(
			self.tk_root,
			text="Confirm Mode!",
			font="MSGothic 16 bold",
			foreground=MAIN_TEXT_COLOR,
			background=MAIN_COMPONENT_COLOR,
			activeforeground=SECONDARY_TEXT_COLOR,
			activebackground=SECONDARY_COMPONENT_COLOR,
			command=self._handle_mode_type_confirmation
		)
		self.confirm_option_btn.grid(row=8, column=0, rowspan=2, columnspan=3, sticky="nsew", padx=50, pady=25)
		
		#
		# For Individual Control Mode
		#
		
		self.intensity_scale_variable = tkinter.DoubleVar()
		self.intensity_scale = tkinter.Scale(
			self.tk_root,
			orient=tkinter.HORIZONTAL,
			variable=self.intensity_scale_variable,
			resolution=0.005,
			from_=0.0,
			to=1.0,
			length=200,
			width=150,
			sliderlength=70,
			borderwidth=5,
			font="MSGothic 16 bold",
			troughcolor=MAIN_COMPONENT_COLOR,
			foreground=MAIN_TEXT_COLOR,
			label="Light Intensity:",
			command=self._send_intensity_scale_update
		)
		self.intensity_scale.grid(row=4, column=4, rowspan=4, columnspan=5, sticky="nsew")
		self.mode_widgets[ModeChange.INDIVIDUAL_CONTROL].append(self.intensity_scale)
		
		#
		# For Morning Countdown Mode
		#
		
		self.hour_scale_variable = tkinter.IntVar()
		self.hour_scale_variable.trace("w", self._trigger_all_threshold_scale_update)
		self.hour_scale = tkinter.Scale(
			self.tk_root,
			variable=self.hour_scale_variable,
			from_=12,
			to=1,
			width=50,
			sliderlength=70,
			font="MSGothic 20 bold",
			troughcolor=MAIN_COMPONENT_COLOR,
			foreground=MAIN_TEXT_COLOR
		)
		self.hour_scale.grid(row=2, column=2, rowspan=3, columnspan=2, sticky="nse", padx=10)
		self.mode_widgets[ModeChange.MORNING_COUNTDOWN].append(self.hour_scale)
		
		self.minute_scale_variable = tkinter.IntVar()
		self.minute_scale_variable.trace("w", self._trigger_all_threshold_scale_update)
		self.minute_scale = tkinter.Scale(
			self.tk_root,
			variable=self.minute_scale_variable,
			from_=59,
			to=0,
			width=50,
			sliderlength=70,
			font="MSGothic 20 bold",
			troughcolor=MAIN_COMPONENT_COLOR,
			foreground=MAIN_TEXT_COLOR
		)
		self.minute_scale.grid(row=2, column=4, rowspan=3, columnspan=2, sticky="nsw", padx=10)
		self.mode_widgets[ModeChange.MORNING_COUNTDOWN].append(self.minute_scale)
		
		self.am_pm_variable = tkinter.StringVar()
		self.am_pm_variable.trace("w", self._trigger_all_threshold_scale_update)
		self.am_radio_btn = tkinter.Radiobutton(
			self.tk_root,
			variable=self.am_pm_variable,
			text="AM",
			value="AM",
			font="MSGothic 16 bold",
			foreground=MAIN_TEXT_COLOR,
			background=MAIN_COMPONENT_COLOR,
			activeforeground=SECONDARY_TEXT_COLOR,
			activebackground=SECONDARY_COMPONENT_COLOR,
			indicatoron=0
		)
		self.am_radio_btn.grid(row=4, column=2, rowspan=2, columnspan=2, sticky="se", padx=10, pady=10)
		self.mode_widgets[ModeChange.MORNING_COUNTDOWN].append(self.am_radio_btn)
		
		self.pm_radio_btn = tkinter.Radiobutton(
			self.tk_root,
			variable=self.am_pm_variable,
			text="PM",
			value="PM",
			font="MSGothic 16 bold",
			foreground=MAIN_TEXT_COLOR,
			background=MAIN_COMPONENT_COLOR,
			activeforeground=SECONDARY_TEXT_COLOR,
			activebackground=SECONDARY_COMPONENT_COLOR,
			indicatoron=0
		)
		self.pm_radio_btn.grid(row=4, column=4, rowspan=2, columnspan=2, sticky="sw", padx=10, pady=10)
		self.mode_widgets[ModeChange.MORNING_COUNTDOWN].append(self.pm_radio_btn)
		
		self.portion_for_green_frame = tkinter.Frame(self.tk_root)
		self.portion_for_green_frame.grid(row=2, column=5, rowspan=2, columnspan=3)
		self.mode_widgets[ModeChange.MORNING_COUNTDOWN].append(self.portion_for_green_frame)
		
		self.portion_for_green_scale_variable = tkinter.DoubleVar()
		self.portion_for_green_scale = tkinter.Scale(
			self.portion_for_green_frame,
			orient=tkinter.HORIZONTAL,
			variable=self.portion_for_green_scale_variable,
			resolution=0.01,
			from_=0.0,
			to=1.0,
			width=35,
			sliderlength=70,
			font="MSGothic 12 bold",
			troughcolor="#3CCB2C",
			foreground=MAIN_TEXT_COLOR
		)
		self.portion_for_green_scale_variable.trace("w", self._handle_green_threshold_scale_update)
		self.portion_for_green_scale.grid(row=0, column=0, sticky="sew")
		
		self.portion_for_green_result = tkinter.Text(
			self.portion_for_green_frame,
			font="MSGothic 12 bold",
			foreground=MAIN_TEXT_COLOR,
			selectforeground=MAIN_TEXT_COLOR,
			background=MAIN_COMPONENT_COLOR,
			highlightbackground=MAIN_COMPONENT_COLOR,
			selectbackground=MAIN_COMPONENT_COLOR,
			insertbackground=MAIN_COMPONENT_COLOR,
			height=1,
			width=35,
			wrap=tkinter.NONE
		)
		self.portion_for_green_result.grid(row=1, column=0, sticky="new")
		
		self.portion_for_yellow_frame = tkinter.Frame(self.tk_root)
		self.portion_for_yellow_frame.grid(row=4, column=5, rowspan=2, columnspan=3)
		self.mode_widgets[ModeChange.MORNING_COUNTDOWN].append(self.portion_for_yellow_frame)
		
		self.portion_for_yellow_scale_variable = tkinter.DoubleVar()
		self.portion_for_yellow_scale_variable.trace("w", self._handle_yellow_threshold_scale_update)
		self.portion_for_yellow_scale = tkinter.Scale(
			self.portion_for_yellow_frame,
			orient=tkinter.HORIZONTAL,
			variable=self.portion_for_yellow_scale_variable,
			resolution=0.01,
			from_=0.0,
			to=1.0,
			width=35,
			sliderlength=70,
			font="MSGothic 12 bold",
			troughcolor=LIGHT_YELLOW_COLOR,
			foreground=MAIN_TEXT_COLOR
		)
		self.portion_for_yellow_scale.grid(row=0, column=0, sticky="sew")
		
		self.portion_for_yellow_result = tkinter.Text(
			self.portion_for_yellow_frame,
			font="MSGothic 12 bold",
			foreground=MAIN_TEXT_COLOR,
			selectforeground=MAIN_TEXT_COLOR,
			background=MAIN_COMPONENT_COLOR,
			highlightbackground=MAIN_COMPONENT_COLOR,
			selectbackground=MAIN_COMPONENT_COLOR,
			insertbackground=MAIN_COMPONENT_COLOR,
			height=1,
			width=35,
			wrap=tkinter.NONE
		)
		self.portion_for_yellow_result.grid(row=1, column=0, sticky="new")
		
		self.portion_for_red_frame = tkinter.Frame(self.tk_root)
		self.portion_for_red_frame.grid(row=6, column=5, rowspan=2, columnspan=3)
		self.mode_widgets[ModeChange.MORNING_COUNTDOWN].append(self.portion_for_red_frame)
		
		self.portion_for_red_scale_variable = tkinter.DoubleVar()
		self.portion_for_red_scale_variable.trace("w", self._handle_red_threshold_scale_update)
		self.portion_for_red_scale = tkinter.Scale(
			self.portion_for_red_frame,
			orient=tkinter.HORIZONTAL,
			variable=self.portion_for_red_scale_variable,
			resolution=0.01,
			from_=0.0,
			to=1.0,
			width=35,
			sliderlength=70,
			font="MSGothic 12 bold",
			troughcolor="#CD2828",
			foreground=MAIN_TEXT_COLOR
		)
		self.portion_for_red_scale.grid(row=0, column=0, sticky="sew")
		
		self.portion_for_red_result = tkinter.Text(
			self.portion_for_red_frame,
			font="MSGothic 12 bold",
			foreground=MAIN_TEXT_COLOR,
			selectforeground=MAIN_TEXT_COLOR,
			background=MAIN_COMPONENT_COLOR,
			highlightbackground=MAIN_COMPONENT_COLOR,
			selectbackground=MAIN_COMPONENT_COLOR,
			insertbackground=MAIN_COMPONENT_COLOR,
			height=1,
			width=35,
			wrap=tkinter.NONE
		)
		self.portion_for_red_result.grid(row=1, column=0, sticky="new")
		
		self.traffic_light_image = None
		
		self.confirm_countdown_btn = tkinter.Button(
			self.tk_root,
			text="Start Countdown!",
			font="MSGothic 16 bold",
			foreground=MAIN_TEXT_COLOR,
			background=MAIN_COMPONENT_COLOR,
			activeforeground=SECONDARY_TEXT_COLOR,
			activebackground=SECONDARY_COMPONENT_COLOR,
			command=self._handle_countdown_confirmation
		)
		self.confirm_countdown_btn.grid(row=9, column=3, rowspan=2, columnspan=5, sticky="nsew", padx=10, pady=30)
		self.mode_widgets[ModeChange.MORNING_COUNTDOWN].append(self.confirm_countdown_btn)
		
		#
		# For Wave Mode
		#
		
		self.wave_update_scale_variable = tkinter.DoubleVar()
		self.wave_update_scale_variable.set(WAVE_PERIOD_DEFAULT_S)
		self.wave_update_scale = tkinter.Scale(
			self.tk_root,
			orient=tkinter.HORIZONTAL,
			variable=self.wave_update_scale_variable,
			resolution=0.01,
			from_=0.1,
			to=60.0,
			length=200,
			width=150,
			sliderlength=70,
			borderwidth=5,
			font="MSGothic 16 bold",
			troughcolor=MAIN_COMPONENT_COLOR,
			foreground=MAIN_TEXT_COLOR,
			label="Wave Period:",
			command=self._send_wave_update_scale_update
		)
		self.wave_update_scale.grid(row=4, column=4, rowspan=4, columnspan=5, sticky="nsew")
		self.mode_widgets[ModeChange.WAVE].append(self.wave_update_scale)
		
		#
		# Done
		#
		
		self.initialized = True
		
		self.handle_mode_type_change()
		
		self.tk_root.attributes("-fullscreen", True)
		self.tk_root.bind('<Escape>', self._close_window)
		
		self.window_is_closing = False
		self.hub_controller.start()
	
	## The routine to perform when the window received a close event/request.
	#
	#  @param self The object pointer.
	#  @param event The event describing the close request. This value is unused.
	def _close_window(self, event):
		self.window_is_closing = True
		
		self.hub_controller.send_mode_type(ModeChange.FULL_OFF, False)
		sleep(0.5)
		
		self.hub_controller.stop()
		self.tk_root.destroy()
	
	## Queue a normalized intensity value. This is called whenever the double variable
	#  mapped to the scale is changed.
	#
	#  @param self The object pointer.
	#  @param event The event describing the value change. This value is unused.
	def _send_intensity_scale_update(self, event):
		self.hub_controller.request_intensity_change(self.intensity_scale_variable.get())
	
	## Queue a wave period value. This is called whenever the double variable
	#  mapped to the scale is changed.
	#
	#  @param self The object pointer.
	#  @param event The event describing the value change. This value is unused.
	def _send_wave_update_scale_update(self, event):
		self.hub_controller.request_wave_update_change(self.wave_update_scale_variable.get())
	
	## Handle when one of the three traffic light time threshold scales change. This is
	#  called by one of the three event handlers for each of the green, yellow, and red
	#  scales.
	#
	#  @param self The object pointer.
	#  @param portion_variable The variable mapped to the scale's value.
	#  @param text_to_update The resulting time to format.
	#  @param must_be_before The variable mapped to the scale that must correspond to a
	#  time before the value described by portion_variable.
	#  @param must_be_after The variable mapped to the scale that must correspond to a
	#  time after the value described by portion_variable.
	def _handle_generic_threshold_scale_update(
		self,
		portion_variable,
		text_to_update,
		must_be_before,
		must_be_after
	):
		local_time, total_mins_remaining = self.hub_controller.get_mins_until(
			self.hour_scale_variable.get(),
			self.minute_scale_variable.get(),
			self.am_pm_variable.get()
		)
		
		mins_til_threshold = portion_variable * total_mins_remaining
		hours_til_threshold, mins_til_threshold = divmod(mins_til_threshold, 60)
		
		final_hour = local_time.tm_hour + hours_til_threshold
		final_min  = local_time.tm_min  + mins_til_threshold
		
		if final_min >= 60:
			final_min  = final_min % 60
			final_hour = final_hour + 1
		
		final_hour = final_hour % 24
		
		final_am_pm_str = "AM" if final_hour < 12 else "PM"
		
		final_hour = final_hour % 12
		if final_hour == 0:
			final_hour = 12
		
		final_hour_str = str(int(final_hour))
		final_min_str  = str(int(final_min))
		
		if len(final_hour_str) == 1:
			final_hour_str = "0" + final_hour_str
		
		if len(final_min_str) == 1:
			final_min_str = "0" + final_min_str
		
		text_to_update.delete("1.0", tkinter.END)
		text_to_update.insert(tkinter.END, final_hour_str + ":" + final_min_str + " " + final_am_pm_str)
		
		if must_be_before:
			if must_be_before.get() > portion_variable:
				must_be_before.set(portion_variable)
		
		if must_be_after:
			if must_be_after.get() < portion_variable:
				must_be_after.set(portion_variable)
	
	## The event callback for if the threshold corresponding to the green light is changed.
	#
	#  @param self The object pointer.
	#  @param args The event arguments. This value is unused.
	def _handle_green_threshold_scale_update(self, *args):
		if self.initialized:
			self._handle_generic_threshold_scale_update(
				self.portion_for_green_scale_variable.get(),
				self.portion_for_green_result,
				None,
				self.portion_for_yellow_scale_variable
			)
	
	## The event callback for if the threshold corresponding to the yellow light is changed.
	#
	#  @param self The object pointer.
	#  @param args The event arguments. This value is unused.
	def _handle_yellow_threshold_scale_update(self, *args):
		if self.initialized:
			self._handle_generic_threshold_scale_update(
				self.portion_for_yellow_scale_variable.get(),
				self.portion_for_yellow_result,
				self.portion_for_green_scale_variable,
				self.portion_for_red_scale_variable
			)
	
	## The event callback for if the threshold corresponding to the red light is changed.
	#
	#  @param self The object pointer.
	#  @param args The event arguments. This value is unused.
	def _handle_red_threshold_scale_update(self, *args):
		if self.initialized:
			self._handle_generic_threshold_scale_update(
				self.portion_for_red_scale_variable.get(),
				self.portion_for_red_result,
				self.portion_for_yellow_scale_variable,
				None
			)
	
	## Handler for when the event that triggers all of the variables corresponding to any
	#  light to be updated.
	#
	#  @param self The object pointer.
	#  @param args The event arguments. This value is unused.
	def _trigger_all_threshold_scale_update(self, *args):
		if self.initialized:
			self._handle_green_threshold_scale_update()
			self._handle_yellow_threshold_scale_update()
			self._handle_red_threshold_scale_update()
	
	## Handler for when the event that triggers when a new mode type is confirmed with the
	#  submit button.
	#
	#  @param self The object pointer.
	def _handle_mode_type_confirmation(self):
		full_selection_string = self.mode_selection_dropdown_variable.get()
		selection_int = int(full_selection_string[:full_selection_string.find(":")])
		self.hub_controller.send_mode_type(selection_int, True)
	
	## Handler for when the light thresholds are confirmed and submitted. This sets the
	#  controller's active mode and passes on the times selected.
	#
	#  @param self The object pointer.
	def _handle_countdown_confirmation(self):
		threshold_times = []
		for result in [
			self.portion_for_green_result,
			self.portion_for_yellow_result,
			self.portion_for_red_result
		]:
			text = result.get("1.0", "end-1c")
			colon_idx = text.find(":")
			space_idx = text.find(" ")
			threshold_times.append((
				int(text[:colon_idx]),
				int(text[colon_idx+1:space_idx]),
				text[space_idx+1:]
			))
		
		self.hub_controller.set_active_sequence_for_mode_type(
			ModeChange.MORNING_COUNTDOWN,
			(
				self.hour_scale_variable.get(),
				self.minute_scale_variable.get(),
				self.am_pm_variable.get()
			),
			*threshold_times
		)
	
	## This starts the Tkinter main loop, and then blocks until the shutdown sequence has
	#  started by attempting to join all of the active threads.
	#
	#  @param self The object pointer.
	def do_main_loop(self):
		self.tk_root.mainloop()
		self.hub_controller.block_until_shutdown()
	
	## The callback for the controller's dedicated to updating the clock.
	#
	#  @param self The object pointer.
	#  @param day_label_text The new text to display for the day.
	#  @param time_label_text The new text to display for the time.
	def handle_clock_change(self, day_label_text, time_label_text):
		if self.window_is_closing:
			return
		
		self.main_canvas.itemconfig(
			self.current_day_time_label,
			text=day_label_text + ", " + time_label_text
		)
	
	## The callback to change the visible widgets when the mdode type changes.
	#
	#  @param self The object pointer.
	def handle_mode_type_change(self):
		current_mode = self.hub_controller.hub_node.current_mode
		text, color  = get_mode_characteristics(current_mode)
		
		self.main_canvas.itemconfig(
			self.current_mode_value_label,
			text=text,
			fill=color
		)
		
		for k, l in self.mode_widgets.items():
			if k == current_mode:
				for w in l:
					w.grid()
			else:
				for w in l:
					w.grid_remove()
		
		if current_mode == ModeChange.MORNING_COUNTDOWN:
			next_hour, next_min, next_am_pm = self.hub_controller.get_local_time()
			
			if next_min >= 45:
				next_min = 0
				if next_hour == 12:
					next_hour  = 1
					next_am_pm = "AM" if next_am_pm == "PM" else "PM"
				else:
					next_hour = next_hour + 1
			else:
				next_min = int((next_min / 15) + 1) * 15
			
			self.minute_scale_variable.set(next_min)
			self.hour_scale_variable.set(next_hour)
			self.am_pm_variable.set(next_am_pm)
			
			self.portion_for_green_scale_variable.set(0.30)
			self.portion_for_yellow_scale_variable.set(0.60)
			self.portion_for_red_scale_variable.set(0.80)
			
			self.draw_traffic_light_for("none")
		else:
			self.main_canvas.delete(TRAFFIC_LIGHT_CANVAS_IMAGE_TAG)
	
	## Depending on the countdown state, draw the corresponding traffic light image.
	#
	#  @param self The object pointer.
	#  @param state_str A string describing the current state.
	def draw_traffic_light_for(self, state_str):
		self.traffic_light_image = ImageTk.PhotoImage(generate_traffic_light_image_for(state_str))
		self.main_canvas.create_image(
			(self.screen_width, self.screen_height - TEXT_INDENTATION),
			image=self.traffic_light_image,
			anchor=tkinter.SE,
			tags=TRAFFIC_LIGHT_CANVAS_IMAGE_TAG
		)
	
	## Given the countdown state code, draw the proper image for the traffic light.
	#
	#  @param self The object pointer.
	#  @param state The ROS constant describing the new state.
	def handle_traffic_light_change(self, state):
		if state == CountdownState.CONFIRMATION:
			return
		
		state_str = None
		
		if state == CountdownState.TO_NONE:
			state_str = "none"
		elif state == CountdownState.TO_GREEN:
			state_str = "green"
		elif state == CountdownState.TO_YELLOW:
			state_str = "yellow"
		elif state == CountdownState.TO_RED:
			state_str = "red"
		elif state == CountdownState.TO_ALL:
			state_str = "all"
		else:
			raise ValueError("Illegal countdown state: [{0}]".format(state))
		
		self.draw_traffic_light_for(state_str)