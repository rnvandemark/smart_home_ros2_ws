# SmartHome

I'm making myself a smart home apartment, which will sit on top of ROS2 to communicate with peripheral
devices around the apartment.

## Installing

1.) Install Git
	sudo apt install git

2.) Install ROS2 Dashing
	sudo apt update && sudo apt install curl gnupg2 lsb-release
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8
	export CHOOSE_ROS_DISTRO=dashing
	sudo apt update
	sudo apt install ros-$CHOOSE_ROS_DISTRO-desktop
	sudo apt install python3-argcomplete
	echo "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" >> ~/.bashrc

3.) Install Colcon
	sudo apt install python3-colcon-common-extensions

4.) Install TKinter
	sudo apt-get install python3-tk
	sudo apt-get install python3-pil python3-pil.imagetk

5.) Download the Project
	mkdir SmartHome && cd SmartHome
	git clone https://github.com/rnvandemark/SmartHome.git src/

6.) Building the Project
	cd SmartHome/
	colcon build

7.) Running the SmartHome Hub
	ros2 run hub hub_main