#include <ros2arduino.h>

//#include "smart_home_msgs/msg/countdown_state.hpp"

#define XRCEDDS_PORT Serial
#define PROGRAM_BAUD 9600

#define INTENSITY_LED 7

#define ANALOG_RESOLUTION_BITS 12

const static int ANALOG_NUM_VALUES = pow(2, ANALOG_RESOLUTION_BITS);

void intensity_change_callback(std_msgs::Float32* msg, void* arg)
{
  (void)(arg);
  
  int fixed_value = (int)(msg->data * ANALOG_NUM_VALUES);
  if (fixed_value >= ANALOG_NUM_VALUES)
  {
    fixed_value = ANALOG_NUM_VALUES - 1;
  }
  else if (fixed_value < 0)
  {
    fixed_value = 0;
  }
  
  analogWrite(INTENSITY_LED, fixed_value);
  digitalWrite(LED_BUILTIN, 1 - digitalRead(LED_BUILTIN));
}

class DueNode : public ros2::Node
{
//private:
//  ros2::Publisher<std_msgs::String>* due_debug_pub;

public:
  DueNode() : Node("due_node")
  {
    this->createSubscriber<std_msgs::Float32>(
      "smart_home/intensity_change_chatter",
      (ros2::CallbackFunc)intensity_change_callback,
      nullptr
    );
    
    //due_debug_pub =
    //  this->createPublisher<std_msgs::String>("smart_home/due_debug_chatter");
  }
};

void setup()
{
  XRCEDDS_PORT.begin(PROGRAM_BAUD);
  while (!XRCEDDS_PORT);
  
  pinMode(INTENSITY_LED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  analogWriteResolution(ANALOG_RESOLUTION_BITS);
  
  ros2::init(&XRCEDDS_PORT);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1500);
  
  static DueNode due_node;
  
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  
  ros2::spin(&due_node);
}
