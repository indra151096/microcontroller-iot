#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <BluetoothSerial.h>
#include <EEPROM.h>
#include <WiFi.h>

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
void connectToNetwork(char * data);

float temperature = 0, humidity = 0;
DHT dht(DHT_PIN, DHT_TYPE);
int lastTransmit = 0;
byte buff[2];
unsigned short lux = 0;
bool autoBrightness = true;
int dutyCycle = 0;
BluetoothSerial SerialBT;
String receivedString = "";
char readDataSSID[EEPROM_SIZE_SSID_PASS], receivedDataSSID[EEPROM_SIZE_SSID_PASS];
portMUX_TYPE gpioIntMux = portMUX_INITIALIZER_UNLOCKED;
bool switchOn = true;
int dataIndex = 0;
bool resetWifi = false;

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

  Serial.println("");
  //init EEPROM
  EEPROM.begin(EEPROM_SIZE_SSID_PASS);
  delay(100);
  readEEPROM(EEPROM_ADDRESS_SSID_PASS, readDataSSID, EEPROM_SIZE_SSID_PASS);
  if ((readDataSSID != NULL) && (readDataSSID[0] == '\0')) {
    Serial.println("---------------------------");
    Serial.println("WiFi Configuration Is Empty");
    Serial.println("Enter Bluetooth Mode");
    Serial.println("---------------------------");
    //enter bluetooth
    SerialBT.begin("ESP32 BT Classic");
  }
  else {
    //enter wifi
    Serial.println("---------------------------");
    Serial.println("Enter WiFi Mode");
    Serial.println("EEPROM DATA SSID PASS: ");
    connectToNetwork(readDataSSID);
    Serial.println("---------------------------");
  }
}

void loop() {
  if (Serial.available()) {
    receivedDataSSID[dataIndex] = Serial.read();
    dataIndex++;

    if (receivedDataSSID[dataIndex-1] == '\n') {
      dataIndex = 0;
      writeEEPROM(EEPROM_ADDRESS_SSID_PASS, receivedDataSSID, EEPROM_SIZE_SSID_PASS);
      //clear receivedData after write
      memset(receivedDataSSID, 0, EEPROM_SIZE_SSID_PASS);
    }
  }

  if (resetWifi) {
    //clear ssid pass in eeprom
    memset(receivedDataSSID, 0, EEPROM_SIZE_SSID_PASS);
    writeEEPROM(EEPROM_ADDRESS_SSID_PASS, receivedDataSSID, EEPROM_SIZE_SSID_PASS);
  
    delay(200);

    portENTER_CRITICAL(&gpioIntMux);
    resetWifi = false;
    portEXIT_CRITICAL(&gpioIntMux);
    Serial.println("---------------------------");
    Serial.println("WiFi Cleared");
    Serial.println("---------------------------");
  }

  lightSensorRequest(LIGHT_SENSOR_ADDRESS);
  delay(200);

  if (autoBrightness == HIGH) {
    checkLightIntensity();
    adjustLed();
  }
  else {
    //adjust PWM LED MAX
    ledcWrite(LED_CHANNEL, 255);
  }

  if (SerialBT.available()) {
    char receivedChar = SerialBT.read();
    receivedString += receivedChar;

    if (receivedChar == '\n') {
      if (receivedString == "TEMP\r\n") {
        updateDhtData();
        SerialBT.println(String(temperature));
      }
      else if (receivedString == "HUMID\r\n") {
        updateDhtData();
        SerialBT.println(String(humidity));
      }
      else if (receivedString == "LUX\r\n") {
        checkLightIntensity();
        SerialBT.println(String(lux));
      }
      else if (receivedString == "AUTOBRIGHT,ON\r\n") {
        autoBrightness = HIGH;
        setIndicatorAutoBrightness(autoBrightness);
        SerialBT.println("ACK");
      }
      else if (receivedString == "AUTOBRIGHT,OFF\r\n") {
        autoBrightness = LOW;
        setIndicatorAutoBrightness(autoBrightness);
        SerialBT.println("ACK");
      }
      else {
        SerialBT.println("Command not found");
      }

      receivedString = "";
      SerialBT.flush();
    }
  }
}

void checkLightIntensity() {
  if (lightSensorGetData(LIGHT_SENSOR_ADDRESS) == LIGHT_SENSOR_DATA_LEN) {
    lux = (((unsigned short)buff[0] << 8) | (unsigned short)buff[1]) /1.2;
    String print = "Nilai intensitas: " + String(lux) + " lux";
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
  resetWifi = true;
  portEXIT_CRITICAL(&gpioIntMux);
}

void readEEPROM(int address, char * data, int EEPROM_SIZE) {
  for (int i  = 0; i < EEPROM_SIZE; i++) {
    data[i] = EEPROM.read(address + i);
  }
}

void writeEEPROM(int address, char * data, int EEPROM_SIZE) {
  Serial.println("Start Writting");
  for (int i  = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(address + i, data[i]);
  }
  EEPROM.commit();
}

void connectToNetwork(char * data) {
  int counter = 0;
  char *ssid;
  char *password;
  char * split = strtok (data,";");

  while (split != NULL) {
    if (counter == 0) {
      Serial.print("SSID: ");
      ssid = split;
      Serial.println(ssid);
    }
    else if (counter == 1) {
      Serial.print("Password: ");
      password = split;
      Serial.println(password);
    }
    counter++;
    split = strtok (NULL, ";");
  }

  WiFi.begin(ssid, password);
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