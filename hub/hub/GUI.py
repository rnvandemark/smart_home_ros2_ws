import tkinter
from PIL import Image, ImageTk
from time import sleep

from ament_index_python.packages import get_package_prefix
from smart_home_msgs.msg import ModeChange

from SmartHomeHubController import SmartHomeHubController

MODES_DICT = {
	ModeChange.FULL_OFF          : ("Full Off",           "#0F2F2F"),
	ModeChange.FULL_ON           : ("Full On",            "#3CCB2C"),
	ModeChange.INDIVIDUAL_CONTROL: ("Individual Control", "#F0E68C"),
	ModeChange.MORNING_COUNTDOWN : ("Morning Countdown",  "#F0E68C"),
	ModeChange.FOLLOW_SOUND      : ("Follow Sound",       "#F0E68C"),
	ModeChange.WAVE              : ("Wave",               "#F0E68C"),
}

MAIN_TEXT_ENABLED_COLOR  = "#652828"
MAIN_TEXT_DISABLED_COLOR = "#220808"

MAIN_COMPONENT_ENABLED_COLOR  = "#CDCDCD"
MAIN_COMPONENT_DISABLED_COLOR = "#686868"

TEXT_INDENTATION = 15

MAIN_CANVAS_GRID_ROW_COUNT = 10
MAIN_CANVAS_GRID_COLUMN_COUNT = 10

def get_mode_characteristics(val):
	if val in MODES_DICT:
		return MODES_DICT[val]
	else:
		return ("UNDEFINED", "#FF0000")

def get_pixel_at(row, col, sw, sh, dx=0, dy=0):
	return (
		int(((sw / MAIN_CANVAS_GRID_COLUMN_COUNT) * col) + dx),
		int(((sh / MAIN_CANVAS_GRID_ROW_COUNT) * row) + dy)
	)

class GUI():
	
	def __init__(self, args):
		self.hub_controller = SmartHomeHubController(args, self.handle_mode_type_change, self.handle_clock_change)
		
		self.tk_root = tkinter.Tk()
		
		screen_width = self.tk_root.winfo_screenwidth()
		screen_height = self.tk_root.winfo_screenheight()
		
		original_image = Image.open(get_package_prefix("hub") + "/lib/hub/resources/kyoto.png")
		original_image = original_image.resize((screen_width, screen_height), Image.ANTIALIAS)
		self.background_image = ImageTk.PhotoImage(original_image)
		
		self.main_canvas = tkinter.Canvas(
			self.tk_root,
			width=screen_width,
			height=screen_height,
			highlightthickness=0
		)
		self.main_canvas.grid(
			row=0,
			column=0,
			rowspan=MAIN_CANVAS_GRID_ROW_COUNT,
			columnspan=MAIN_CANVAS_GRID_COLUMN_COUNT
		)
		
		self.main_canvas.create_image((0, 0), image=self.background_image, anchor=tkinter.NW)
		
		greetings_label = self.main_canvas.create_text(
			(TEXT_INDENTATION, 0),
			text="Hey there, beautiful ;)",
			font="MSGothic 42 bold",
			fill=MAIN_TEXT_ENABLED_COLOR,
			anchor=tkinter.NW
		)
		
		current_mode_label = self.main_canvas.create_text(
			(screen_width - TEXT_INDENTATION, 0),
			text="Current Mode:",
			font="MSGothic 20 bold",
			fill=MAIN_TEXT_ENABLED_COLOR,
			anchor=tkinter.NE
		)
		
		self.current_mode_value_label = self.main_canvas.create_text(
			(screen_width - TEXT_INDENTATION, 40),
			font="MSGothic 20 bold",
			anchor=tkinter.NE
		)
		
		self.current_day_label = self.main_canvas.create_text(
			(TEXT_INDENTATION, screen_height - (TEXT_INDENTATION + 40)),
			font="MSGothic 24 bold",
			fill=MAIN_TEXT_ENABLED_COLOR,
			anchor=tkinter.SW
		)
		
		self.current_time_label = self.main_canvas.create_text(
			(TEXT_INDENTATION, screen_height - TEXT_INDENTATION),
			font="MSGothic 24 bold",
			fill=MAIN_TEXT_ENABLED_COLOR,
			anchor=tkinter.SW
		)
		
		self.intensity_scale_variable = tkinter.DoubleVar()
		self.intensity_scale = tkinter.Scale(
			self.tk_root,
			orient=tkinter.HORIZONTAL,
			variable=self.intensity_scale_variable,
			resolution=0.005,
			from_=0.0,
			to=1.0,
			length=200,
			width=50,
			font="MSGothic 10 bold",
			troughcolor=MAIN_COMPONENT_ENABLED_COLOR,
			foreground=MAIN_TEXT_ENABLED_COLOR,
			command=self._send_intensity_scale_update
		)
		self.intensity_scale.grid(row=7, column=2)
		
		intensity_scale_grid_info = self.intensity_scale.grid_info()
		self.intensity_scale_label = self.main_canvas.create_text(
			get_pixel_at(
				intensity_scale_grid_info["row"],
				intensity_scale_grid_info["column"],
				screen_width,
				screen_height,
				dx=20
			),
			text="Light Intensity: ",
			font="MSGothic 20 bold",
			fill=MAIN_TEXT_ENABLED_COLOR,
			anchor=tkinter.NE
		)
		
		self.handle_mode_type_change()
		
		self.tk_root.attributes("-fullscreen", True)
		self.tk_root.bind('<Escape>', self._close_window)
		
		self.closed_window = False
		self.hub_controller.start()
	
	def _close_window(self, event):
		self.closed_window = True
		
		self.hub_controller.mode_change_node.send_mode_type(ModeChange.FULL_OFF, False)
		sleep(0.5)
		
		self.hub_controller.stop()
		self.tk_root.destroy()
	
	def _send_intensity_scale_update(self, event):
		self.hub_controller.request_intensity_change(self.intensity_scale_variable.get())
	
	def handle_clock_change(self, day_label_text, time_label_text):
		if self.closed_window:
			return
		
		self.main_canvas.itemconfig(
			self.current_day_label,
			text=day_label_text
		)
		
		self.main_canvas.itemconfig(
			self.current_time_label,
			text=time_label_text
		)
	
	def do_main_loop(self):
		self.tk_root.mainloop()
		self.hub_controller.block_until_shutdown()
	
	def handle_mode_type_change(self):
		current_mode = self.hub_controller.mode_change_node.current_mode
		text, color  = get_mode_characteristics(current_mode)
		
		self.main_canvas.itemconfig(
			self.current_mode_value_label,
			text=text,
			fill=color
		)
		
		if current_mode == ModeChange.INDIVIDUAL_CONTROL:
			self.intensity_scale.config(
				troughcolor=MAIN_COMPONENT_ENABLED_COLOR,
				foreground=MAIN_TEXT_ENABLED_COLOR,
				state=tkinter.NORMAL,
				takefocus=1
			)
			self.main_canvas.itemconfig(
				self.intensity_scale_label,
				fill=MAIN_TEXT_ENABLED_COLOR
			)
		else:
			self.intensity_scale.config(
				troughcolor=MAIN_COMPONENT_DISABLED_COLOR,
				foreground=MAIN_TEXT_DISABLED_COLOR,
				state=tkinter.DISABLED,
				takefocus=0
			)
			self.main_canvas.itemconfig(
				self.intensity_scale_label,
				fill=MAIN_TEXT_DISABLED_COLOR
			)