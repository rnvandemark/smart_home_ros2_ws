#!/bin/bash

########################################################################################
# A small helper function to flash the Arduino and then, if successful, proceed with   #
# the standard Colcon build.                                                           #
########################################################################################

./src/uc_colcon/tools/arduino_install.sh
RETURN_VAL=$?
if [ $RETURN_VAL -ne 0 ]
then
	>&2 echo "Error flashing Arduino: Error code $RETURN_VAL"
	exit 1
fi

colcon build
exit $?