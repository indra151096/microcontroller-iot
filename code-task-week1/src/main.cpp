#include <Arduino.h>

#define LED_PIN 4
#define LED_PIN_2 18
#define LED_PIN_3 19

//function declaration
void turnOnOffLED(uint8_t pin);

void setup() {
  // put your setup code here, to run once:
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char data = Serial.read();
    Serial.print("Karakter yang dikirim dari laptop: ");
    Serial.println(data);

    if (data == '1') {
      turnOnOffLED(BUILTIN_LED);
    }
    else if (data == '2') {
      turnOnOffLED(LED_PIN);
    }
    else if (data == '3') {
      turnOnOffLED(LED_PIN_2);
    }
    else if (data == '4') {
      turnOnOffLED(LED_PIN_3);
    }
    else {
      Serial.println("Command tidak ditemukan");
    }
  }
}


//function definition
void turnOnOffLED(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delay(2000);
  digitalWrite(pin, LOW);
}