def main(args=None):
	from GUI import GUI
	gui = GUI(args)
	gui.do_main_loop()
	
if __name__ == "__main__":
	main()