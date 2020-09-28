#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <w25q64.hpp>

#define PIN_SWITCH_AUTO_BRIGHTNESS 13
#define PIN_LED_1 14
#define PIN_LED_2 27
#define PIN_LED_3 26
#define PIN_LED_4 25

#define LIGHT_SENSOR_ADDRESS 0x23
#define LIGHT_SENSOR_DATA_LEN 2

void checkLightIntensity();
void adjustLed(int numberOfLedOn);
void lightSensorRequest(int address);
int lightSensorGetData(int address);
bool readDataBrigthness();
void writeDataBrightness(bool autoBrightness);
void setIndicatorAutoBrightness(bool autoBrightness);

byte buff[2];
unsigned short lux = 0;

//flash memory
unsigned char writePage[256] = "";
unsigned char readPage[256] = "";
byte chipId[4] = "";

uint8_t arrayOfPinLed[4] = {PIN_LED_1, PIN_LED_2, PIN_LED_3, PIN_LED_4};
int totalLed = 4;
bool autoBrightness = true;
bool switchOn = true;

w25q64 spiChip;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  //for indicator auto brightness status
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(PIN_SWITCH_AUTO_BRIGHTNESS, INPUT_PULLUP);
  
  //flash memory
  spiChip.begin();
  spiChip.getId(chipId);
  Serial.print("chip ID in bytes: ");
  for (int i = 0; i < LEN_ID; i++) {
    Serial.print(chipId[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  autoBrightness = readDataBrigthness();
  setIndicatorAutoBrightness(autoBrightness);
}

void loop() {
  lightSensorRequest(LIGHT_SENSOR_ADDRESS);
  delay(200);

  if (autoBrightness) {
    checkLightIntensity();
  }
  else {
    adjustLed(4);
  }
  
  if (digitalRead(PIN_SWITCH_AUTO_BRIGHTNESS) == LOW) {
    autoBrightness = !autoBrightness;
    Serial.println("Auto Brightness: " + String(autoBrightness));
    //update flash memory
    writeDataBrightness(autoBrightness);
    setIndicatorAutoBrightness(autoBrightness);
  }

  delay(1000);
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

void writeDataBrightness(bool autoBrightness) {
  if (autoBrightness)
    memcpy(writePage, "1", sizeof("1"));
  else
    memcpy(writePage, "0", sizeof("0"));

  spiChip.erasePageSector(0xFFFF);
  spiChip.pageWrite(writePage, 0xFFFF);
  Serial.println("Done Writing");
  delay(1000);
}

bool readDataBrigthness() {
  char data;
  char off = '0';
  Serial.println("Start Reading");
  spiChip.readPages(readPage, 0xFFFF, 1);
  Serial.print("Data read from chip: ");
  if (readPage[0] > 7 && readPage[0] < 127) {
    data = (char)readPage[0];
    Serial.print(data);
  }
  Serial.println();
  if (data == off)
    return false;
  else
    return true;
}

void setIndicatorAutoBrightness(bool autoBrightness) {
  Serial.println("SET INDICATOR LIGHT " + String(autoBrightness));
  if (autoBrightness) {
    digitalWrite(BUILTIN_LED, HIGH);
  }
  else {
    digitalWrite(BUILTIN_LED, LOW);
  }
}