import tkinter

from SmartHomeHubThread import SmartHomeHubThread

GUI_REFRESH_TIMEOUT_S = 0.05

class GUI():
	
	def __init__(self, args):
		self.hub_thread = SmartHomeHubThread(args)
		self.hub_thread.start()
		
		self.tk_root = tkinter.Tk()
		w = tkinter.Label(self.tk_root, text="Hello Tkinter!")
		w.pack()
		self.tk_root.mainloop()
		
		self.hub_thread.join()