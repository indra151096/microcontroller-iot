#include <Arduino.h>

#define POT_PIN 13
#define PIN_LED_1 14
#define PIN_LED_2 27
#define PIN_LED_3 26
#define PIN_LED_4 25

int adcValue = 0;
float celcius = 0;

uint8_t currentPinOn = 0;

//definition
void turnOnlyOneLed(uint8_t pinGoingToOn);
void checkLed(float celcius);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:'
  adcValue = analogRead(POT_PIN);
  Serial.println("------------------------------");
  String printData = "Nilai ADC: " + String(adcValue);
  Serial.println(printData);
  celcius = ((float) adcValue / 4095.0) * 100.0;
  printData = "Celcius: " + String(celcius) + " *C";
  Serial.println(printData);
  Serial.println("------------------------------");

  checkLed(celcius);

  delay(1000);
}

//declaration
void checkLed(float celcius) {
  if (celcius < 15) {
    turnOnlyOneLed(PIN_LED_1);
  }
  else if (celcius <= 25) {
    turnOnlyOneLed(PIN_LED_2);
  }
  else if (celcius <= 30) {
    turnOnlyOneLed(PIN_LED_3);
  }
  else {
    turnOnlyOneLed(PIN_LED_4);
  }
}


void turnOnlyOneLed(uint8_t pinGoingToOn) {
  //turn off led only when any led on before
  if (currentPinOn != 0) {
    digitalWrite(currentPinOn, LOW);
  }
  
  digitalWrite(pinGoingToOn, HIGH);
  currentPinOn = pinGoingToOn;
}