# This message is to notify the traffic light of a transition to a different phase in the countdown sequence.
# Though this only sends an unsigned byte, this message was defined for the sake of using ROS constants.

uint8 CONFIRMATION=1
uint8 TO_NONE=2
uint8 TO_GREEN=3
uint8 TO_YELLOW=4
uint8 TO_RED=5
uint8 TO_ALL=6

uint8 state