#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <BluetoothSerial.h>

#define DHT_PIN 18
#define DHT_TYPE DHT11
#define LED_PIN 14
#define LED_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define LIGHT_SENSOR_ADDRESS 0x23
#define LIGHT_SENSOR_DATA_LEN 2

//definition
void updateDhtData();
void checkLightIntensity();
void lightSensorRequest(int address);
int lightSensorGetData(int address);
void setIndicatorAutoBrightness(char autoBrightness);
void adjustLed();

float temperature = 0, humidity = 0;
DHT dht(DHT_PIN, DHT_TYPE);
int lastTransmit = 0;
byte buff[2];
unsigned short lux = 0;
bool autoBrightness = true;
int dutyCycle = 0;
BluetoothSerial SerialBT;
String receivedString = "";

void setup() {
  //configure LED PWM
  ledcSetup(LED_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  //attach channel
  ledcAttachPin(LED_PIN, LED_CHANNEL);

  //for indicator auto brightness status
  pinMode(BUILTIN_LED, OUTPUT);

  SerialBT.begin("ESP32 BT Classic");
  dht.begin();
  Serial.begin(9600);
  Wire.begin();

  setIndicatorAutoBrightness(autoBrightness);
}

void loop() {
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