# This message is to notify all auxilary devices that listen to the smart home's mode.
# This tells the auxilary devices the new mode to be in.

uint8 FULL_OFF=1
uint8 FULL_ON=2
uint8 INDIVIDUAL_CONTROL=3
uint8 MORNING_COUNTDOWN=4
uint8 FOLLOW_SOUND=5
uint8 WAVE=6

std_msgs/Header header
uint8 mode_type