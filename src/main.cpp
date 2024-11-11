#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <.env.h>
#include <env_defaults.h>

uint8_t mac[] = {0xDE, 0xAD, MAC_4_LAST};
const IPAddress selfIp(SELF_IP);
const IPAddress mqttServerIp(MQTT_SERVER_IP);

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

uint32_t mqttLastReconnectAttemptAt = 0;

uint32_t lastPresenceAt = 0;
uint32_t lastStatInAt = 0;
uint32_t lastStatGrpAt = 0;

uint8_t publishStatIn = 0x00;
uint8_t publishStatGrp = 0x00;

uint32_t startAutoCloseInAt;
uint32_t startAutoCloseGrpAt;

uint32_t autoCloseInTime;
uint32_t autoCloseGrpTime;

uint8_t runAutoCloseIn = 0x00;
uint8_t runAutoCloseGrp = 0x00;

uint16_t filterReleaseIn = FILTER_RELEASE_STEPS;
uint16_t filterReleaseOut = FILTER_RELEASE_STEPS;
uint16_t filterReleaseGrp = FILTER_RELEASE_STEPS;
uint16_t filterAttackIn = FILTER_ATTACK_STEPS;
uint16_t filterAttackOut = FILTER_ATTACK_STEPS;
uint16_t filterAttackGrp = FILTER_ATTACK_STEPS;
uint8_t runAttackIn = 0x00;
uint8_t runAttackOut = 0x00;
uint8_t runAttackGrp = 0x00;

#define SENS_MASK (B_SENS_IN | B_SENS_OUT | B_SENS_GRP)
#define LED_MASK (B_FB_LED_IN | B_FB_LED_OUT | B_FB_LED_GRP)
#define RLY_MASK (B_FB_RLY_IN | B_FB_RLY_GRP)
#define FB_MASK (LED_MASK | RLY_MASK)

#ifdef WATCHDOG_EN
  Watchdog watchdog;
#endif

#define PUBLISH_STAT_IN publishStatIn = 0xff;
#define PUBLISH_STAT_GRP publishStatGrp = 0xff;

#define CLOSE_IN \
  PORT_FB &= ~B_FB_RLY_IN; \
  runAutoCloseIn = 0x00; \
  PUBLISH_STAT_IN;

#define OPEN_IN \
  PORT_FB |= B_FB_RLY_IN; \
  runAutoCloseIn = 0x00; \
  PUBLISH_STAT_IN;

#define CLOSE_GRP \
  PORT_FB &= ~B_FB_RLY_GRP; \
  runAutoCloseGrp = 0x00; \
  PUBLISH_STAT_GRP;

#define OPEN_GRP \
  PORT_FB |= B_FB_RLY_GRP; \
  runAutoCloseGrp = 0x00; \
  PUBLISH_STAT_GRP;

uint8_t getPayloadSec(uint8_t* payload, unsigned int length){
  uint8_t iii;
  uint8_t sec;
  sec = 0;
  for (iii = 0; iii < length; iii++){
    sec *= 10;
    sec += payload[iii] - '0';
  }
  return sec;
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  #ifdef SERIAL_EN
    uint8_t iii;
    Serial.print("sub rec: ");
    Serial.print(topic);
    Serial.print(" : ");
    for (iii = 0; iii < length; iii++) {
      Serial.print((char) payload[iii]);
    }
  #endif

  if (strcmp(topic, SUB_GET_IN) == 0){
    PUBLISH_STAT_IN;
    return;
  }

  if (strcmp(topic, SUB_GET_GRP) == 0){
    PUBLISH_STAT_GRP;
    return;
  }

  if (strcmp(topic, SUB_OPEN_IN) == 0){
    OPEN_IN;
    return;
  }

  if (strcmp(topic, SUB_CLOSE_IN) == 0){
    CLOSE_IN;
    return;
  }

  if (strcmp(topic, SUB_OPEN_GRP) == 0){
    OPEN_GRP;
    return;
  }

  if (strcmp(topic, SUB_CLOSE_GRP) == 0){
    CLOSE_GRP;
    return;
  }

  if (strcmp(topic, SUB_ONCE_IN) == 0){
    if (!length){
      return;
    }
    OPEN_IN;
    startAutoCloseInAt = millis();
    autoCloseInTime = getPayloadSec(payload, length) * 1000;
    runAutoCloseIn = 0xff;
    return;
  }

  if (strcmp(topic, SUB_ONCE_GRP) == 0){
    if (!length){
      return;
    }
    OPEN_GRP;
    startAutoCloseGrpAt = millis();
    autoCloseGrpTime = getPayloadSec(payload, length) * 1000;
    runAutoCloseGrp = 0xff;
    return;
  }
}

inline void sens(){
  if (filterReleaseIn){
    filterReleaseIn--;
  } else {
    PORT_FB |= B_FB_LED_IN;
  }

  if (PIN_SENS & B_SENS_IN){
    runAttackIn = 0x00;
  } else {
    if (!filterReleaseIn){
      runAttackIn = 0xff;
      filterAttackIn = FILTER_ATTACK_STEPS;
    }
    filterReleaseIn = FILTER_RELEASE_STEPS;
  }

  if (runAttackIn){
    filterAttackIn--;
    if (!filterAttackIn){
      runAttackIn = 0x00;

      if (runAutoCloseIn){
        CLOSE_IN;
      }
      #ifdef ETH_EN
      mqttClient.publish(PUB_SENS_IN, "1");
      #endif
      #ifdef SERIAL_EN
      Serial.print(PUB_SENS_IN);
      Serial.println(": 1");
      #endif
      PORT_FB &= ~B_FB_LED_IN;
    }
  }

  ///

  if (filterReleaseOut){
    filterReleaseOut--;
  } else {
    PORT_FB |= B_FB_LED_OUT;
  }

  if (PIN_SENS & B_SENS_OUT){
    runAttackOut = 0x00;
  } else {
    if (!filterReleaseOut){
      runAttackOut = 0xff;
      filterAttackOut = FILTER_ATTACK_STEPS;
    }
    filterReleaseOut = FILTER_RELEASE_STEPS;
  }

  if (runAttackOut){
    filterAttackOut--;
    if (!filterAttackOut){
      runAttackOut = 0x00;
      #ifdef ETH_EN
      mqttClient.publish(PUB_SENS_OUT, "1");
      #endif
      #ifdef SERIAL_EN
      Serial.print(PUB_SENS_OUT);
      Serial.println(": 1");
      #endif
      PORT_FB &= ~B_FB_LED_OUT;
    }
  }

  ////

  if (filterReleaseGrp){
    filterReleaseGrp--;
  } else {
    PORT_FB |= B_FB_LED_GRP;
  }

  if (PIN_SENS & B_SENS_GRP){
    runAttackGrp = 0x00;
  } else {
    if (!filterReleaseGrp){
      runAttackGrp = 0xff;
      filterAttackGrp = FILTER_ATTACK_STEPS;
    }
    filterReleaseGrp = FILTER_RELEASE_STEPS;
  }

  if (runAttackGrp){
    filterAttackGrp--;
    if (!filterAttackGrp){
      runAttackGrp = 0x00;

      if (runAutoCloseGrp){
        CLOSE_GRP;
      }
      #ifdef ETH_EN
      mqttClient.publish(PUB_SENS_GRP, "1");
      #endif
      #ifdef SERIAL_EN
      Serial.print(PUB_SENS_GRP);
      Serial.println(": 1");
      #endif
      PORT_FB &= ~B_FB_LED_GRP;
    }
  }
}

bool mqttReconnect() {
  uint8_t iii;
  uint8_t charPos;
  char clientId[10] = CLIENT_ID_PREFIX;
  for (iii = 0; iii < 4; iii++){
    charPos = strlen(clientId);
    clientId[charPos] = '0' + random(0, 10);
    clientId[charPos + 1] = '\0';
  }
  if (mqttClient.connect(clientId)) {
    mqttClient.subscribe(SUB_GET_IN);
    mqttClient.subscribe(SUB_GET_GRP);
    mqttClient.subscribe(SUB_OPEN_IN);
    mqttClient.subscribe(SUB_CLOSE_IN);
    mqttClient.subscribe(SUB_OPEN_GRP);
    mqttClient.subscribe(SUB_CLOSE_GRP);
    mqttClient.subscribe(SUB_ONCE_IN);
    mqttClient.subscribe(SUB_ONCE_GRP);
  }
  return mqttClient.connected();
}

inline void readCommandFromSerial() {
  char ch;
  if (!Serial.available()){
    return;
  }
  ch = Serial.read();

  switch (ch){
    case '2':
      Serial.println("cmd 2 publish stat in");
      PUBLISH_STAT_IN;
      break;
    case '3':
      Serial.println("cmd 3 publish stat grp");
      PUBLISH_STAT_GRP;
      break;
    case '4':
      Serial.println("cmd 4 open grp");
      OPEN_GRP;
      break;
    case '5':
      Serial.println("cmd 5 close grp");
      CLOSE_GRP;
      break;
    case '6':
      Serial.println("cmd 6 open once grp 14 sec");
      OPEN_GRP;
      startAutoCloseGrpAt = millis();
      autoCloseGrpTime = 14000;
      runAutoCloseGrp = 0xff;
      break;
    case '7':
      Serial.println("cmd 7 open in");
      OPEN_IN;
      break;
    case '8':
      Serial.println("cmd 8 close in");
      CLOSE_IN;
      break;
    case '9':
      Serial.println("cmd 9 once in 14 sec");
      OPEN_IN;
      startAutoCloseInAt = millis();
      autoCloseInTime = 14000;
      runAutoCloseIn = 0xff;
      break;
    default:
      Serial.println("cmd not found");
      break;
  }
  return;
}

void setup() {

  delay(250);

  // GPIO inputs
  PORT_SENS |= SENS_MASK; // weak pull-up
  DDR_SENS &= ~SENS_MASK;
  // GPIO outputs
  PORT_FB &= ~LED_MASK;
  PORT_FB |= RLY_MASK;
  DDR_FB |= FB_MASK;

  #ifdef ETH_EN
  mqttClient.setServer(mqttServerIp, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  Ethernet.init(ETH_CS_PIN);
  SPI.begin();
  Ethernet.begin(mac, selfIp);
  #endif

  #ifdef SERIAL_EN
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  #endif

  delay(500);

  PORT_FB |= LED_MASK; // switch off LEDS

  delay(1000);

  #ifdef ETH_EN
  if (Ethernet.hardwareStatus() == EthernetHardwareStatus::EthernetNoHardware) {
    #ifdef SERIAL_EN
      Serial.println("W5500 not found.");
    #endif
    while(1){
      delay(1);
    }
  }

  if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
    #ifdef SERIAL_EN
      Serial.println("Ethernet not connected.");
    #endif
  } else {
    #ifdef SERIAL_EN
      Serial.println("Ethernet ok.");
    #endif
  }

  mqttReconnect();
  #endif

  #ifdef WATCHDOG_EN
    watchdog.enable(Watchdog::TIMEOUT_4S);
  #endif

  publishStatIn = 0xff;
  publishStatGrp = 0xff;
}

void loop() {
  #ifdef WATCHDOG_EN
    watchdog.reset();
  #endif
  #ifdef ETH_EN
  Ethernet.maintain();


  if (mqttClient.connected()) {
  #endif
    sens();

    if (!publishStatIn){
      if (millis() - lastStatInAt > STAT_IN_TIME){
        publishStatIn = 0xff;
      }
    }

    if (!publishStatGrp){
      if (millis() - lastStatGrpAt > STAT_GRP_TIME){
        publishStatGrp = 0xff;
      }
    }

    sens();

    if (publishStatIn){
      #ifdef ETH_EN
      mqttClient.publish(PUB_STAT_IN, PORT_FB & B_FB_RLY_IN ? "open" : "closed");
      #endif
      #ifdef SERIAL_EN
      Serial.print(PUB_STAT_IN);
      Serial.println(PORT_FB & B_FB_RLY_IN ? ": open" : ": closed");
      #endif
      lastStatInAt = millis();
      publishStatIn = 0x00;
    } else if (publishStatGrp){
      #ifdef ETH_EN
      mqttClient.publish(PUB_STAT_GRP, PORT_FB & B_FB_RLY_GRP ? "open" : "closed");
      #endif
      #ifdef SERIAL_EN
      Serial.print(PUB_STAT_GRP);
      Serial.println(PORT_FB & B_FB_RLY_GRP ? ": open" : ": closed");
      #endif
      lastStatGrpAt = millis();
      publishStatGrp = 0x00;
    } else if (millis() - lastPresenceAt > PRESENCE_TIME){
      #ifdef ETH_EN
      mqttClient.publish(PUB_PRESENCE, "1");
      #endif
      #ifdef SERIAL_EN
      Serial.print(PUB_PRESENCE);
      Serial.println(": 1");
      #endif
      lastPresenceAt = millis();
    }

    sens();

    if (runAutoCloseIn && (millis() - startAutoCloseInAt > autoCloseInTime)){
      CLOSE_IN;
    }

    if (runAutoCloseGrp && (millis() - startAutoCloseGrpAt > autoCloseGrpTime)){
      CLOSE_GRP;
    }

    sens();

    #ifdef SERIAL_EN
      readCommandFromSerial();
    #endif

    #ifdef ETH_EN
    mqttClient.loop();

  } else {

    if (millis() - mqttLastReconnectAttemptAt > MQTT_CONNECT_RETRY_TIME) {
      if (mqttReconnect()) {
        #ifdef SERIAL_EN
          Serial.println("connected");
        #endif
      } else {
        #ifdef SERIAL_EN
          Serial.print("failed, rc=");
          Serial.print(mqttClient.state());
          Serial.print(" try again in ");
          Serial.print(MQTT_CONNECT_RETRY_TIME);
          Serial.println(" ms.");
        #endif
        mqttLastReconnectAttemptAt = millis();
      }
    }
  }
  #endif
}