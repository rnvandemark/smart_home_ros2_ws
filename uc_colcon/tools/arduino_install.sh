#!/bin/bash

########################################################################################
# Uses the Arduino Command Line Interface (CLI) to compile the sketch and upload the   #
# resulting assembly file to the target COMM port.                                     #
#                                                                                      #
# The working directory for this script should be the in the src directory for the     #
# ROS2 workspace. There is an optional argument to describe the target COMM port. If   #
# unprovided, "/dev/ttyACM0" is the default URL to attempt to write to. Furthermore,   #
# this script assumes an Arduino Due is the target controller.                         #
########################################################################################

SKETCH_URL="${PWD}/src/uc_colcon/sketches/TestCLI"

COMM_PORT_URL=""
if [ $# -gt 0 ]
then
	COMM_PORT_URL=$1
else
	COMM_PORT_URL="/dev/ttyACM0"
fi

arduino-cli compile --fqbn arduino:sam:arduino_due_x "$SKETCH_URL"
if [ $? -ne 0 ]
then
	exit 1
fi

arduino-cli upload -p $COMM_PORT_URL --fqbn arduino:sam:arduino_due_x "$SKETCH_URL"
if [ $? -ne 0 ]
then
	exit 2
fi

exit 0