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

uint8_t autoClose = 0x00;

uint32_t startAutoCloseInAt;
uint32_t startAutoCloseGrpAt;

uint32_t autoCloseInTime;
uint32_t autoCloseGrpTime;

uint8_t filterCountIn = FILTER_COUNT_START;
uint8_t filterCountOut = FILTER_COUNT_START;
uint8_t filterCountGrp = FILTER_COUNT_START;

#define SENS_MASK (B_SENS_IN | B_SENS_OUT | B_SENS_GRP)
#define LED_MASK (B_FB_LED_IN | B_FB_LED_OUT | B_FB_LED_GRP)
#define RLY_MASK (B_FB_RLY_IN | B_FB_RLY_GRP)
#define FB_MASK (LED_MASK | RLY_MASK)

#ifdef WATCHDOG_EN
  Watchdog watchdog;
#endif

#define PUBLISH_STAT_IN publishStatIn = 0x01;
#define PUBLISH_STAT_GRP publishStatGrp = 0x01;

#define CLOSE_IN \
  PORT_FB &= ~B_FB_RLY_IN; \
  autoClose &= ~B_FB_RLY_IN; \
  PUBLISH_STAT_IN;

#define OPEN_IN \
  PORT_FB |= B_FB_RLY_IN; \
  autoClose &= ~B_FB_RLY_IN; \
  PUBLISH_STAT_IN;

#define CLOSE_GRP \
  PORT_FB &= ~B_FB_RLY_GRP; \
  autoClose &= ~B_FB_RLY_GRP; \
  PUBLISH_STAT_GRP;

#define OPEN_GRP \
  PORT_FB |= B_FB_RLY_GRP; \
  autoClose &= ~B_FB_RLY_GRP; \
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
    autoClose |= B_FB_RLY_IN;
    return;
  }

  if (strcmp(topic, SUB_ONCE_GRP) == 0){
    if (!length){
      return;
    }
    OPEN_GRP;
    startAutoCloseGrpAt = millis();
    autoCloseGrpTime = getPayloadSec(payload, length) * 1000;
    autoClose |= B_FB_RLY_GRP;
    return;
  }
}

inline void sens(){
  if (filterCountIn){
    filterCountIn--;
    PORT_FB &= ~B_FB_LED_IN;
  } else {
    PORT_FB |= B_FB_LED_IN;
  }

  if (!(PIN_SENS & B_SENS_IN)){
    if (!filterCountIn){
      if (autoClose & B_SENS_IN){
        CLOSE_IN;
      }
      mqttClient.publish(PUB_SENS_IN, "1");
    }
    filterCountIn = FILTER_COUNT_START;
  }

  if (filterCountOut){
    PORT_FB &= ~B_FB_LED_OUT;
    filterCountOut--;
  } else {
    PORT_FB |= B_FB_LED_OUT;
  }

  if (!(PIN_SENS & B_SENS_OUT)){
    if (!filterCountOut){
      mqttClient.publish(PUB_SENS_OUT, "1");
    }
    filterCountOut = FILTER_COUNT_START;
  }

  if (filterCountGrp){
    filterCountGrp--;
    PORT_FB &= ~B_FB_LED_GRP;
  } else {
    PORT_FB |= B_FB_LED_GRP;
  }

  if (!(PIN_SENS & B_SENS_GRP)){
    if (!filterCountGrp){
      if (autoClose & B_SENS_GRP){
        CLOSE_GRP;
      }
      mqttClient.publish(PUB_SENS_GRP, "1");
    }
    filterCountGrp = FILTER_COUNT_START;
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

void setup() {

  delay(250);

  // GPIO inputs
  DDR_SENS &= ~SENS_MASK;
  // GPIO outputs
  PORT_FB &= ~LED_MASK;
  PORT_FB |= RLY_MASK;
  DDR_FB |= FB_MASK;
  mqttClient.setServer(mqttServerIp, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  Ethernet.init(ETH_CS_PIN);
  SPI.begin();

  Ethernet.begin(mac, selfIp);

  #ifdef SERIAL_EN
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  #endif

  delay(500);

  PORT_FB |= LED_MASK; // switch off LEDS

  delay(1000);

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

  #ifdef WATCHDOG_EN
    watchdog.enable(Watchdog::TIMEOUT_4S);
  #endif
}

void loop() {
  #ifdef WATCHDOG_EN
    watchdog.reset();
  #endif

  Ethernet.maintain();

  if (mqttClient.connected()) {

    sens();

    if (publishStatIn || (millis() - lastStatInAt > STAT_IN_TIME)){
      mqttClient.publish(PUB_STAT_IN, PORT_FB & B_FB_RLY_IN ? "open" : "closed");
      lastStatInAt = millis();
      publishStatIn = 0x00;
    } else if (publishStatGrp || (millis() - lastStatGrpAt > STAT_GRP_TIME)){
      mqttClient.publish(PUB_STAT_GRP, PORT_FB & B_FB_RLY_GRP ? "open" : "closed");
      lastStatGrpAt = millis();
      publishStatGrp = 0x00;
    } else if (millis() - lastPresenceAt > PRESENCE_TIME){
      mqttClient.publish(PUB_PRESENCE, "1");
      lastPresenceAt = millis();
    }

    sens();

    if (autoClose){
      if ((autoClose & B_FB_RLY_IN) && (millis() - startAutoCloseInAt > autoCloseInTime)){
        CLOSE_IN;
      }
      if ((autoClose & B_FB_RLY_GRP) && (millis() - startAutoCloseGrpAt > autoCloseGrpTime)){
        CLOSE_GRP;
      }
    }

    sens();

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
}