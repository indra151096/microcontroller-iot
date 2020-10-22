#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

#define PIN_SWITCH_AUTO_BRIGHTNESS 13
#define PIN_LED_1 14
#define PIN_LED_2 27
#define PIN_LED_3 26
#define PIN_LED_4 25
#define LIGHT_SENSOR_ADDRESS 0x23
#define LIGHT_SENSOR_DATA_LEN 2
#define EEPROM_SIZE_LED 1
#define EEPROM_SIZE_SSID_PASS 64
#define EEPROM_ADDRESS_LED 0
#define EEPROM_ADDRESS_SSID_PASS 32

//definiton
void IRAM_ATTR gpioISR();
void readEEPROM(int address, char * data, int EEPROM_SIZE);
void writeEEPROM(int address, char * data, int EEPROM_SIZE);
void writeEEPROMChar(int address, char data);
void checkLightIntensity();
void adjustLed(int numberOfLedOn);
void lightSensorRequest(int address);
int lightSensorGetData(int address);
void setIndicatorAutoBrightness(char autoBrightness);
char readDataBrigthness();
void printSSIDPass(char * data);

char readDataSSID[EEPROM_SIZE_SSID_PASS], receivedDataSSID[EEPROM_SIZE_SSID_PASS];
char readDataLED[EEPROM_SIZE_LED], receivedDataLED[EEPROM_SIZE_LED];
int dataIndex = 0;

bool changeLedStatus = false;
portMUX_TYPE gpioIntMux = portMUX_INITIALIZER_UNLOCKED;

byte buff[2];
unsigned short lux = 0;
uint8_t arrayOfPinLed[4] = {PIN_LED_1, PIN_LED_2, PIN_LED_3, PIN_LED_4};
int totalLed = 4;
char autoBrightness = LOW;
bool switchOn = true;

void setup() {
  Serial.begin(9600);

  Wire.begin();

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  //for indicator auto brightness status
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(PIN_SWITCH_AUTO_BRIGHTNESS, INPUT_PULLUP);

  attachInterrupt(PIN_SWITCH_AUTO_BRIGHTNESS, &gpioISR, RISING);

  //init EEPROM
  EEPROM.begin(EEPROM_SIZE_SSID_PASS);
  delay(100);
  readEEPROM(EEPROM_ADDRESS_SSID_PASS, readDataSSID, EEPROM_SIZE_SSID_PASS);
  Serial.println("EEPROM DATA SSID PASS: ");
  printSSIDPass(readDataSSID);
  autoBrightness = EEPROM.read(EEPROM_ADDRESS_LED);
  if (autoBrightness == LOW) {
    Serial.println("Auto Brightness: OFF");
  }
  else {
    Serial.println("Auto Brightness: ON");
  }
  setIndicatorAutoBrightness(autoBrightness);
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

  lightSensorRequest(LIGHT_SENSOR_ADDRESS);
  delay(200);

  if (autoBrightness == HIGH) {
    checkLightIntensity();
  }
  else {
    adjustLed(4);
  }

  if (changeLedStatus) {
    autoBrightness = !autoBrightness;
    if (autoBrightness == LOW) {
      Serial.println("Auto Brightness: OFF");
    }
    else {
      Serial.println("Auto Brightness: ON");
    }

    delay(200);

    portENTER_CRITICAL(&gpioIntMux);
    changeLedStatus = false;
    portEXIT_CRITICAL(&gpioIntMux);

    writeEEPROMChar(EEPROM_ADDRESS_LED, autoBrightness);
    setIndicatorAutoBrightness(autoBrightness);
  }
}

void IRAM_ATTR gpioISR() {
  portENTER_CRITICAL(&gpioIntMux);
  changeLedStatus = true;
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

void writeEEPROMChar(int address, char data) {
  Serial.println("Start Writting");
  EEPROM.write(address, data);
  EEPROM.commit();
}

void checkLightIntensity() {
    if (lightSensorGetData(LIGHT_SENSOR_ADDRESS) == LIGHT_SENSOR_DATA_LEN) {
      lux = (((unsigned short)buff[0] << 8) | (unsigned short)buff[1]) /1.2;
      String print = "Nilai intensitas: " + String(lux) + " lux";
      Serial.println(print);

      if (lux <= 250) {
        adjustLed(4);
      }
      else if (lux <= 500) {
        adjustLed(3);
      }
      else if (lux <= 750) {
        adjustLed(2);
      }
      else if (lux <= 1000) {
        adjustLed(1);
      }
      else {
        adjustLed(0);
      }
  }
}

void adjustLed(int numberOfLedOn) {
  for (int i = 0; i < totalLed; i++) {
    if (i < numberOfLedOn) {
      digitalWrite(arrayOfPinLed[i], HIGH);
    }
    else {
      digitalWrite(arrayOfPinLed[i], LOW);
    }
  }
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

void printSSIDPass(char * data) {
  int counter = 0;
  char * split = strtok (data,";");
  while (split != NULL) {
    if (counter == 0) {
      Serial.print("SSID: ");
    }
    else if (counter == 1) {
      Serial.print("Password: ");
    }
    counter++;

    Serial.println(split);
    split = strtok (NULL, ";");
  }
}