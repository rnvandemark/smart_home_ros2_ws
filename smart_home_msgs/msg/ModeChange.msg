# This message is to notify all auxilary devices that listen to the smart home's mode.
# This tells the auxilary devices the new mode to be in.

uint8 NONE=0
uint8 MORNING_COUNTDOWN=0

std_msgs/Header header
uint8 mode_type