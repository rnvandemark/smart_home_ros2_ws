# SmartHome

I'm making myself a smart home apartment, which will sit on top of ROS2 to communicate with peripheral
devices around the apartment. That's all I've got.

## Installing

+ Install Git
	- sudo apt install git

+ Install ROS2 Dashing Patch #6
	- sudo locale-gen en_US en_US.UTF-8
	- sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	- export LANG=en_US.UTF-8
	- sudo apt update && sudo apt install curl gnupg2 lsb-release
	- curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	- sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
	- sudo apt update && sudo apt install -y build-essential cmake git python3-colcon-common-extensions python3-pip python-rosdep python3-vcstool wget
	- python3 -m pip install -U argcomplete flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest pytest-cov pytest-runner setuptools
	- sudo apt install --no-install-recommends -y libasio-dev libtinyxml2-dev
	- sudo apt install --no-install-recommends -y libcunit1-dev
	- mkdir -p <location_dir>/ros2_dashing_ws/src
	- cd <location_dir>/ros2_dashing_ws
	- wget https://raw.githubusercontent.com/ros2/ros2/release-dashing-20200319/ros2.repos
	- vcs import src < ros2.repos
	- sudo rosdep init
	- rosdep update
	- rosdep install --from-paths src --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"

+ Install Colcon
	- sudo apt install python3-colcon-common-extensions

+ Install TKinter
	- sudo apt-get install python3-tk
	- sudo apt-get install python3-pil python3-pil.imagetk

+ Download the Project
	- cd <location_dir>/
	- git clone https://github.com/rnvandemark/smart_home_ws.git

+ Building ROS2
	- cd <location_dir>/ros2_dashing_ws
	- colcon build
	- echo "source <location_dir>/ros2_dashing_ws/install/setup.bash" >> ~/.bashrc

+ Building the Project
	- cd <location_dir>/smart_home_ws/
	- colcon build

+ Running the SmartHome Hub
	- ros2 run hub hub_main
