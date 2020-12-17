#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#define DHT_PIN 18
#define DHT_TYPE DHT11
#define LED_PIN 14
#define LED_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define LIGHT_SENSOR_ADDRESS 0x23
#define LIGHT_SENSOR_DATA_LEN 2
#define EEPROM_SIZE_SSID_PASS 64
#define EEPROM_ADDRESS_SSID_PASS 32
#define PIN_SWITCH 13
#define PUBLISH_INTERVAL 10000

//definition
void updateDhtData();
void checkLightIntensity();
void lightSensorRequest(int address);
int lightSensorGetData(int address);
void setIndicatorAutoBrightness(char autoBrightness);
void adjustLed();
void IRAM_ATTR gpioISR();
void readEEPROM(int address, char * data, int EEPROM_SIZE);
void writeEEPROM(int address, char * data, int EEPROM_SIZE);
void connectToNetwork();
void publishMessage();
void connectToMqtt();
void mqttCallback(char *topic, byte *payload, long length);
void do_actions(const char *message);

float temperature = 0, humidity = 0;
DHT dht(DHT_PIN, DHT_TYPE);
int lastTransmit = 0;
byte buff[2];
unsigned short lux = 0;
bool autoBrightness = true;
int dutyCycle = 0;
String receivedString = "";
char readDataSSID[EEPROM_SIZE_SSID_PASS], receivedDataSSID[EEPROM_SIZE_SSID_PASS];
portMUX_TYPE gpioIntMux = portMUX_INITIALIZER_UNLOCKED;
bool switchOn = true;
int dataIndex = 0;
bool resetWifi = false;
String ssid = "IS";
String pass = "nusantech";
String mqttServer = "broker.hivemq.com";
String mqttPort = "1883";
String deviceId = "HG1";
String pubTopic = String(deviceId + "/sensor_data");
String subTopic = String(deviceId + "/led_status");
WiFiClient ESPClient;
PubSubClient ESPMqtt(ESPClient);
unsigned long lastPublish = 0;

void setup() {
  //configure LED PWM
  ledcSetup(LED_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  //attach channel
  ledcAttachPin(LED_PIN, LED_CHANNEL);

  //for indicator auto brightness status
  pinMode(BUILTIN_LED, OUTPUT);

  pinMode(PIN_SWITCH, INPUT_PULLUP);
  attachInterrupt(PIN_SWITCH, &gpioISR, RISING);

  dht.begin();
  Serial.begin(9600);
  Wire.begin();

  setIndicatorAutoBrightness(autoBrightness);

  //setup MQTT
  WiFi.disconnect();
  ESPMqtt.setServer(mqttServer.c_str(), mqttPort.toInt());
  ESPMqtt.setCallback(mqttCallback);

  Serial.println("");
  //enter wifi
  Serial.println("---------------------------");
  Serial.println("Enter WiFi Mode");
  connectToNetwork();
  Serial.println("---------------------------");

  connectToMqtt();
}

void loop() {

  lightSensorRequest(LIGHT_SENSOR_ADDRESS);
  delay(200);
  checkLightIntensity();

  if (autoBrightness == HIGH) {
    adjustLed();
  }
  else {
    //adjust PWM LED MAX
    ledcWrite(LED_CHANNEL, 255);
  }

  if (millis() - lastPublish > PUBLISH_INTERVAL) {
    //reconnect wifi when needed
    if (WiFi.status() != WL_CONNECTED) {
      connectToNetwork();
    }

    if (WiFi.status() == WL_CONNECTED && !ESPMqtt.connected()) {
      connectToMqtt();
    }
    
    updateDhtData();
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    Serial.print("LUX: ");
    Serial.println(lux);
    Serial.print("PUBLISH MESSAGE...");
    publishMessage();
    lastPublish = millis();
  }
  ESPMqtt.loop();
}

void checkLightIntensity() {
  if (lightSensorGetData(LIGHT_SENSOR_ADDRESS) == LIGHT_SENSOR_DATA_LEN) {
    lux = (((unsigned short)buff[0] << 8) | (unsigned short)buff[1]) /1.2;
    String print = "";
    if (autoBrightness == HIGH) {
      print += "(AUTO ON)";
    }
    else {
      print += "(AUTO OFF)";
    }

    print += "Nilai intensitas: " + String(lux) + " lux";
    Serial.println(print);
  }
}

void adjustLed() {
  if (lux <= 2500) {
    dutyCycle = 255 - (lux/10);
  }
  else {
    dutyCycle = 0;
  }
  //adjust PWM LED
  ledcWrite(LED_CHANNEL, dutyCycle);
}

void lightSensorRequest(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x10);
  Wire.endTransmission();
}

int lightSensorGetData(int address) {
  int i = 0;

  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while (Wire.available())
  {
    buff[i] = Wire.read();
    i++;
  }

  Wire.endTransmission();

  return i;
}

void setIndicatorAutoBrightness(char autoBrightness) {
  if (autoBrightness == HIGH) {
    digitalWrite(BUILTIN_LED, HIGH);
  }
  else {
    digitalWrite(BUILTIN_LED, LOW);
  }
}

void updateDhtData() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}

void IRAM_ATTR gpioISR() {
  portENTER_CRITICAL(&gpioIntMux);
  autoBrightness = !autoBrightness;
  portEXIT_CRITICAL(&gpioIntMux);
}

void connectToNetwork() {
  WiFi.begin(ssid.c_str(), pass.c_str());
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1500);
    Serial.println("Establishing connection to WiFi...");
    tries++;
    if (tries > 3) {
      break;
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Fail to Connect to WiFI");
  }
  else {
    Serial.println("WiFI Connected");
    Serial.println("WiFI Details");
    Serial.println("---------------------------");
    Serial.println(WiFi.macAddress());
    Serial.println(WiFi.localIP());
    Serial.println("---------------------------");
  }
}

void connectToMqtt() {
  while (!ESPMqtt.connected()) {
    Serial.println("ESP connecting to MQTT...");

    if (ESPMqtt.connect("ESP32ClientIS", "","")) {
      Serial.println("Connected to MQTT Server");
      ESPMqtt.subscribe(subTopic.c_str());
    }
    else {
      Serial.print("ERROR failed with state");
      Serial.print(ESPMqtt.state());
      Serial.print("\r\n");
      delay(2000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, long length) {
  char msg[256];

  Serial.print("Message arrived [");
  Serial.print(subTopic);
  Serial.print("]");
  for (int i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }
  do_actions(msg);
}

void do_actions(const char *message) {
  const size_t capacity = JSON_OBJECT_SIZE(2) + 30;
  DynamicJsonDocument doc(capacity);

  deserializeJson(doc, message);

  const char *deviceIdReceive = doc["deviceId"];
  const char *ledStatus = doc["ledStatus"];

  //check device id
  if (String(deviceIdReceive) == deviceId) {
    if (String(ledStatus) == "ON") {
    Serial.println("Turn On LED");
    digitalWrite(BUILTIN_LED, HIGH);
    }
    else if (String(ledStatus) == "OFF") {
      Serial.println("Turn Off LED");
      digitalWrite(BUILTIN_LED, LOW);
    }
    else {
      Serial.println("Could not recognize LED Status");
    }
  }
  else {
    Serial.println("Device not granted!");
  }
}

void publishMessage() {
  char msgToSend[1024] = {0};
  const size_t capacity = JSON_OBJECT_SIZE(4);
  DynamicJsonDocument doc(capacity);

  String tempJson = String(temperature);
  String humidJson = String(humidity);
  String luxJson = String(lux);

  doc["eventName"] = "sesorStatus";
  doc["temp"] = tempJson.c_str();
  doc["humid"] = humidJson.c_str();
  doc["lux"] = luxJson.c_str();

  serializeJson(doc, msgToSend);

  ESPMqtt.publish(pubTopic.c_str(), msgToSend);
}