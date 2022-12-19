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

uint32_t mqttLastReconnectAttempt = 0;

uint8_t blockTrig = 0x00;
uint32_t blockTrigInUntil = 0;
uint32_t blockTrigOutUntil = 0;
uint32_t blockTrigGrpUntil = 0;

uint32_t sensInFilterTime = 0;
uint32_t sensOutFilterTime = 0;
uint32_t sensGrpFilterTime = 0;

uint32_t pulseInValidUntil = 0;
uint32_t pulseOutValidUntil = 0;
uint32_t pulseGrpValidUntil = 0;

uint32_t upInCount = 0;
uint32_t upOutCount = 0;
uint32_t upGrpCount = 0;
char msg[16];

uint32_t lastPresence = 0;
uint32_t lastStatIn = 0;
uint32_t lastStatGrp = 0;

uint8_t lastSens;
uint8_t newSens;
uint8_t changedSens = 0x00;

uint8_t autoClose = 0x00;
uint32_t inAutoCloseAt = 0;
uint32_t grpAutoCloseAt = 0;

#define SENS_MASK (B_SENS_IN | B_SENS_OUT | B_SENS_GRP)
#define LED_MASK (B_FB_LED_IN | B_FB_LED_OUT | B_FB_LED_GRP)
#define RLY_MASK (B_FB_RLY_IN | B_FB_RLY_GRP)
#define FB_MASK (LED_MASK | RLY_MASK)

#ifdef WATCHDOG_EN
  Watchdog watchdog;
#endif

#define GET_IN lastStatIn = 0;
#define GET_GRP lastStatGrp = 0;

#define CLOSE_IN \
  PORT_FB &= ~B_FB_RLY_IN; \
  autoClose &= ~B_FB_RLY_IN; \
  GET_IN;
#define OPEN_IN \
  PORT_FB |= B_FB_RLY_IN; \
  autoClose &= ~B_FB_RLY_IN; \
  GET_IN;
#define CLOSE_GRP \
  PORT_FB &= ~B_FB_RLY_GRP; \
  autoClose &= ~B_FB_RLY_GRP; \
  GET_GRP;
#define OPEN_GRP \
  PORT_FB |= B_FB_RLY_GRP; \
  autoClose &= ~B_FB_RLY_GRP; \
  GET_GRP;

uint32_t getAutoCloseTime(uint8_t* payload, unsigned int length){
  uint8_t iii;
  uint8_t sec;
    sec = 0;
    for (iii = 0; iii < length; iii++){
      sec *= 10;
      sec += payload[iii] - '0';
    }
    return millis() + (sec * 1000);
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
    GET_IN;
    return;
  }

  if (strcmp(topic, SUB_GET_GRP) == 0){
    GET_GRP;
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
    inAutoCloseAt = getAutoCloseTime(payload, length);
    autoClose |= B_FB_RLY_IN;
    return;
  }

  if (strcmp(topic, SUB_ONCE_GRP) == 0){
    if (!length){
      return;
    }
    OPEN_GRP;
    grpAutoCloseAt = getAutoCloseTime(payload, length);
    autoClose |= B_FB_RLY_GRP;
    return;
  }
}

inline void sens(){
  uint32_t mt;
  uint8_t sensDiff;

  mt = millis();

  // release blocks

  if (blockTrig){
    if ((blockTrig & B_SENS_IN) && (mt > blockTrigInUntil)){
      blockTrig &= ~B_SENS_IN;
    }
    if ((blockTrig & B_SENS_OUT) && (mt > blockTrigOutUntil)){
      blockTrig &= ~B_SENS_OUT;
    }
    if ((blockTrig & B_SENS_GRP) && (mt > blockTrigGrpUntil)){
      blockTrig &= ~B_SENS_GRP;
    }
  }

  newSens = PIN_SENS & SENS_MASK;
  sensDiff = newSens^lastSens;

  if (!sensDiff){
    return;
  }

  if (B_SENS_IN & sensDiff & ~blockTrig){
    blockTrig |= B_SENS_IN;
    blockTrigInUntil = mt + SENS_FILTER_TIME;
    itoa(upInCount, msg, 10);
    if (newSens & B_SENS_IN){
      lastSens |= B_SENS_IN;
      PORT_FB |= B_FB_LED_IN;
      mqttClient.publish(PUB_TRIG_IN_UP, msg);
      upInCount++;
      pulseInValidUntil = mt + PULSE_IN_VALID_TIME;
    } else {
      lastSens &= ~B_SENS_IN;
      PORT_FB &= ~B_FB_LED_IN;
      mqttClient.publish(PUB_TRIG_IN_DOWN, msg);
      if (mt < pulseInValidUntil){
        mqttClient.publish(PUB_TRIG_IN_PULSE, msg);
        if (autoClose & B_SENS_IN){
          CLOSE_IN;
        }
      }
    }
  }

  if (B_SENS_OUT & sensDiff & ~blockTrig){
    blockTrig |= B_SENS_OUT;
    blockTrigOutUntil = mt + SENS_FILTER_TIME;
    itoa(upOutCount, msg, 10);
    if (newSens & B_SENS_OUT){
      lastSens |= B_SENS_OUT;
      PORT_FB |= B_FB_LED_OUT;
      mqttClient.publish(PUB_TRIG_OUT_UP, msg);
      upOutCount++;
      pulseOutValidUntil = mt + PULSE_OUT_VALID_TIME; 
    } else {
      lastSens &= ~B_SENS_OUT;
      PORT_FB &= ~B_FB_LED_OUT;
      mqttClient.publish(PUB_TRIG_OUT_DOWN, msg);
      if (mt < pulseOutValidUntil){
        mqttClient.publish(PUB_TRIG_OUT_PULSE, msg);
      }    
    }
  }

  if (B_SENS_GRP & sensDiff & ~blockTrig){
    blockTrig |= B_SENS_GRP;
    blockTrigGrpUntil = mt + SENS_FILTER_TIME;
    itoa(upGrpCount, msg, 10);   
    if (newSens & B_SENS_GRP){
      lastSens |= B_SENS_GRP;
      PORT_FB |= B_FB_LED_GRP;
      mqttClient.publish(PUB_TRIG_GRP_UP, msg);
      upGrpCount++;
      pulseGrpValidUntil = mt + PULSE_GRP_VALID_TIME;
    } else {
      lastSens &= ~B_SENS_GRP;
      PORT_FB &= ~B_FB_LED_GRP;
      mqttClient.publish(PUB_TRIG_GRP_DOWN, msg);
      if (mt < pulseGrpValidUntil){
        mqttClient.publish(PUB_TRIG_GRP_PULSE, msg);
        if (autoClose & B_SENS_GRP){
          CLOSE_GRP;
        }
      }
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
  lastSens = PIN_SENS & SENS_MASK;
  newSens = lastSens;

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
  uint32_t mt;

  #ifdef WATCHDOG_EN
    watchdog.reset();
  #endif
  Ethernet.maintain();

  if (mqttClient.connected()) {

    sens();

    mt = millis();

    if (mt - lastStatIn > STAT_IN_TIME){
      mqttClient.publish(PUB_STAT_IN, PORT_FB & B_FB_RLY_IN ? "open" : "closed");
      lastStatIn = mt;
    } else if (mt - lastStatGrp > STAT_GRP_TIME){
      mqttClient.publish(PUB_STAT_GRP, PORT_FB & B_FB_RLY_GRP ? "open" : "closed");
      lastStatGrp = mt;
    } else if (mt - lastPresence > PRESENCE_TIME){
      mqttClient.publish(PUB_PRESENCE, "1");
      lastPresence = mt;
    }

    sens();

    if (autoClose){
      if ((autoClose & B_FB_RLY_IN) && (inAutoCloseAt < mt)){
        CLOSE_IN;
      }
      if ((autoClose & B_FB_RLY_GRP) && (grpAutoCloseAt < mt)){
        CLOSE_GRP;
      }
    }

    sens();

    mqttClient.loop();
  } else {

    if (millis() - mqttLastReconnectAttempt > MQTT_CONNECT_RETRY_TIME) {
      if (mqttReconnect()) {
        #ifdef SERIAL_EN
          Serial.println("connected");
        #endif
        mqttLastReconnectAttempt = 0;
      } else {
        #ifdef SERIAL_EN
          Serial.print("failed, rc=");
          Serial.print(mqttClient.state());
          Serial.print(" try again in ");
          Serial.print(MQTT_CONNECT_RETRY_TIME);
          Serial.println(" ms.");
        #endif
        mqttLastReconnectAttempt = millis();
      }
    }
  }
}