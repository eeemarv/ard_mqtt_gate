#include <Arduino.h>

/*
* definitions that 
* can be changed
* in .env.h 
*/

#ifndef MAC_4_LAST
  #define MAC_4_LAST 0xbe, 0xef, 0x13, 0x60
#endif
#ifndef SELF_IP
  #define SELF_IP 192, 168, 0, 60
#endif
#ifndef ETH_CS_PIN 
  #define ETH_CS_PIN 10
#endif
#ifndef MQTT_SERVER_IP
  #define MQTT_SERVER_IP 192, 168, 0, 70
#endif
#ifndef MQTT_PORT
  #define MQTT_PORT 1883
#endif
#ifndef MQTT_CONNECT_RETRY_TIME
  #define MQTT_CONNECT_RETRY_TIME 5000
#endif
#ifndef SERIAL_BAUD 
  #define SERIAL_BAUD 9600
#endif

#ifndef SUB_GET_IN
  #define SUB_GET_IN "g/get/in"
#endif
#ifndef SUB_GET_GRP
  #define SUB_GET_GRP "g/get/grp"
#endif

#ifndef SUB_OPEN_IN
  #define SUB_OPEN_IN "g/open/in"
#endif 
#ifndef SUB_CLOSE_IN
  #define SUB_CLOSE_IN "g/close/in"
#endif 
#ifndef SUB_OPEN_GRP
  #define SUB_OPEN_GRP "g/open/grp"
#endif
#ifndef SUB_CLOSE_GRP
  #define SUB_CLOSE_GRP "g/close/grp"
#endif

#ifndef SUB_ONCE_IN 
  #define SUB_ONCE_IN "g/once/in"
#endif
#ifndef SUB_ONCE_GRP
  #define SUB_ONCE_GRP "g/once/grp"
#endif

#ifndef PUB_TRIG_IN_DOWN
  #define PUB_TRIG_IN_DOWN "g/t/in/d"
#endif
#ifndef PUB_TRIG_IN_UP
  #define PUB_TRIG_IN_UP "g/t/in/u"
#endif
#ifndef PUB_TRIG_IN_PULSE
  #define PUB_TRIG_IN_PULSE "g/t/in/p"
#endif

#ifndef PUB_TRIG_OUT_DOWN
  #define PUB_TRIG_OUT_DOWN "g/t/out/d"
#endif
#ifndef PUB_TRIG_OUT_UP
  #define PUB_TRIG_OUT_UP "g/t/out/u"
#endif
#ifndef PUB_TRIG_OUT_PULSE
  #define PUB_TRIG_OUT_PULSE "g/t/out/p"
#endif

#ifndef PUB_TRIG_GRP_DOWN
  #define PUB_TRIG_GRP_DOWN "g/t/grp/d"
#endif
#ifndef PUB_TRIG_GRP_UP
  #define PUB_TRIG_GRP_UP "g/t/grp/u"
#endif
#ifndef PUB_TRIG_GRP_PULSE
  #define PUB_TRIG_GRP_PULSE "g/t/grp/p"
#endif

#ifndef PUB_STAT_IN
  #define PUB_STAT_IN "g/s/in"
#endif
#ifndef PUB_STAT_GRP
  #define PUB_STAT_GRP "g/s/grp"
#endif
#ifndef PUB_PRESENCE
  #define PUB_PRESENCE "g/p"
#endif

#ifndef CLIENT_ID_PREFIX
  #define CLIENT_ID_PREFIX "g_"
#endif
#ifndef SENS_FILTER_TIME
  #define SENS_FILTER_TIME 500
#endif
#ifndef PULSE_IN_VALID_TIME
  #define PULSE_IN_VALID_TIME 5000
#endif
#ifndef PULSE_OUT_VALID_TIME
  #define PULSE_OUT_VALID_TIME 5000
#endif
#ifndef PULSE_GRP_VALID_TIME
  #define PULSE_GRP_VALID_TIME 5000
#endif

#ifndef STAT_IN_TIME 
  #define STAT_IN_TIME 10000
#endif
#ifndef STAT_GRP_TIME 
  #define STAT_GRP_TIME 10000
#endif
#ifndef PRESENCE_TIME 
  #define PRESENCE_TIME 2000
#endif

#ifndef PORT_SENS
  #define PORT_SENS PORTD
  #define DDR_SENS DDRD
  #define PIN_SENS PIND
#endif
#ifndef B_SENS_IN
  #define B_SENS_IN B00000100
#endif
#ifndef B_SENS_OUT
  #define B_SENS_OUT B00001000
#endif
#ifndef B_SENS_GRP
  #define B_SENS_GRP B00010000
#endif

#ifndef PORT_FB
  #define PORT_FB PORTC
  #define DDR_FB DDRC
  #define PIN_FB PINC
#endif
#ifndef B_FB_LED_IN
  #define B_FB_LED_IN B00000010
#endif
#ifndef B_FB_LED_OUT
  #define B_FB_LED_OUT B00000100
#endif
#ifndef B_FB_LED_GRP
  #define B_FB_LED_GRP B00001000
#endif
#ifndef B_FB_RLY_IN
  #define B_FB_RLY_IN B00010000
#endif
#ifndef B_FB_RLY_GRP
  #define B_FB_RLY_GRP B00100000
#endif
