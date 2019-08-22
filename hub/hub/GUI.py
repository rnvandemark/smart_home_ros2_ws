import tkinter
from PIL import Image, ImageTk
from time import sleep, localtime, strftime
from threading import Thread

from ament_index_python.packages import get_package_prefix
from smart_home_msgs.msg import ModeChange

from SmartHomeHubThread import SmartHomeHubThread

MODES_DICT = {
	1: ("Full Off",          "#0F2F2F"),
	2: ("Full On",           "#228B22"),
	3: ("Morning Countdown", "#F0E68C"),
	4: ("Follow Sound",      "#F0E68C"),
	5: ("Wave",              "#F0E68C"),
}

MONTH_ABBREVIATIONS = {
	1: "Jan",
	2: "Feb",
	3: "Mar",
	4: "Apr",
	5: "May",
	6: "Jun",
	7: "Jul",
	8: "Aug",
	9: "Sep",
	10: "Oct",
	11: "Nov",
	12: "Dec"
}

DAYS = {
	0: "Monday",
	1: "Tuesday",
	2: "Wednesday",
	3: "Thursday",
	4: "Friday",
	5: "Saturday",
	6: "Sunday"
}

MAIN_TEXT_COLOR = "#652828"
TEXT_INDENTATION = 15

def get_mode_characteristics(val):
	if val in MODES_DICT:
		return MODES_DICT[val]
	else:
		return ("UNDEFINED", "#FF0000")

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

class GUI():
	
	def __init__(self, args):
		self.hub_thread = SmartHomeHubThread(args, self, self.set_mode_type_label_text)
		self.hub_thread.start()
		
		self.tk_root = tkinter.Tk()
		
		screen_width = self.tk_root.winfo_screenwidth()
		screen_height = self.tk_root.winfo_screenheight()
		
		original_image = Image.open(get_package_prefix("hub") + "/lib/hub/resources/kyoto.png")
		original_image = original_image.resize((screen_width, screen_height), Image.ANTIALIAS)
		self.background_image = ImageTk.PhotoImage(original_image)
		
		self.current_mode_canvas = tkinter.Canvas(
			self.tk_root,
			width=screen_width,
			height=screen_height,
			highlightthickness=0
		)
		self.current_mode_canvas.pack()
		
		self.current_mode_canvas.create_image((0, 0), image=self.background_image, anchor=tkinter.NW)
		
		greetings_label = self.current_mode_canvas.create_text(
			(TEXT_INDENTATION, 0),
			text="Hey there, beautiful ;)",
			font="MSGothic 42 bold",
			fill=MAIN_TEXT_COLOR,
			anchor=tkinter.NW
		)
		
		current_mode_label = self.current_mode_canvas.create_text(
			(screen_width - TEXT_INDENTATION, 0),
			text="Current Mode:",
			font="MSGothic 20 bold",
			fill=MAIN_TEXT_COLOR,
			anchor=tkinter.NE
		)
		
		self.current_mode_value_label = self.current_mode_canvas.create_text(
			(screen_width - TEXT_INDENTATION, 40),
			font="MSGothic 20 bold",
			anchor=tkinter.NE
		)
		self.set_mode_type_label_text()
		
		self.current_day_label = self.current_mode_canvas.create_text(
			(TEXT_INDENTATION, screen_height - (TEXT_INDENTATION + 40)),
			font="MSGothic 20 bold",
			fill=MAIN_TEXT_COLOR,
			anchor=tkinter.SW
		)
		
		self.current_time_label = self.current_mode_canvas.create_text(
			(TEXT_INDENTATION, screen_height - TEXT_INDENTATION),
			font="MSGothic 20 bold",
			fill=MAIN_TEXT_COLOR,
			anchor=tkinter.SW
		)
		
		self.keep_clock_alive = True
		self.clock_thread = Thread(target=self._do_clock_loop)
		self.clock_thread.start()
		
		self.tk_root.attributes("-fullscreen", True)
		self.tk_root.bind('<Escape>', self._close_window)
	
	def _close_window(self, event):
		self.keep_clock_alive = False
		self.clock_thread.join()
		self.tk_root.destroy()
		
		self.hub_thread.mode_change_node.send_mode_type(ModeChange.FULL_OFF, False)
		
		sleep(1)
		
		self.hub_thread.stop()
		self.hub_thread.join()
	
	def _do_clock_loop(self):
		while self.keep_clock_alive:
			local_time = localtime()
			
			self.current_mode_canvas.itemconfig(
				self.current_day_label,
				text=get_formatted_day_str(local_time.tm_wday, local_time.tm_mon, local_time.tm_mday)
			)
			
			self.current_mode_canvas.itemconfig(
				self.current_time_label,
				text=strftime("%H:%M:%S", local_time) + " " + ("AM" if local_time.tm_hour < 12 else "PM")
			)
			
			sleep(0.5)
	
	def do_main_loop(self):
		self.tk_root.mainloop()
		self.hub_thread.join()
	
	def set_mode_type_label_text(self):
		text, color = get_mode_characteristics(self.hub_thread.mode_change_node.current_mode)
		self.current_mode_canvas.itemconfig(
			self.current_mode_value_label,
			text=text,
			fill=color
		)