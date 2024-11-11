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

#ifndef PUB_SENS_IN
  #define PUB_SENS_IN "g/sens/in"
#endif

#ifndef PUB_SENS_OUT
  #define PUB_SENS_OUT "g/sens/out"
#endif

#ifndef PUB_SENS_GRP
  #define PUB_SENS_GRP "g/sens/grp"
#endif

#ifndef PUB_STAT_IN
  #define PUB_STAT_IN "g/stat/in"
#endif
#ifndef PUB_STAT_GRP
  #define PUB_STAT_GRP "g/stat/grp"
#endif
#ifndef PUB_PRESENCE
  #define PUB_PRESENCE "g/p"
#endif

#ifndef CLIENT_ID_PREFIX
  #define CLIENT_ID_PREFIX "g_"
#endif

#ifndef STAT_IN_TIME
  #define STAT_IN_TIME 60000
#endif
#ifndef STAT_GRP_TIME
  #define STAT_GRP_TIME 62222
#endif
#ifndef PRESENCE_TIME
  #define PRESENCE_TIME 10444
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

#ifndef FILTER_RELEASE_STEPS
  #define FILTER_RELEASE_STEPS 32640
#endif

#ifndef FILTER_ATTACK_STEPS
  #define FILTER_ATTACK_STEPS 2048
#endif
