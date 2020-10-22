#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>

#define DHT_PIN 18
#define DHT_TYPE DHT11
#define SERVICE_UUID "f55bb6f4-96dd-458a-a1fc-05c8b12dddaf"
#define TX_CHARACTERISTIC_UUID "3fc5e96d-df5e-4e90-b80d-8028e04159b7"
#define RX_CHARACTERISTIC_UUID "304ae665-bb0b-4191-b5c5-d9ae66f5b9b8"

//definition
void updateDhtData();

float temperature = 0, humidity = 0;
DHT dht(DHT_PIN, DHT_TYPE);
int lastTransmit = 0;
String receivedString;
BLECharacteristic *pTxCharacteristic, *pRxCharacteristic;

class BLECallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.println("**********");
      Serial.print("Received String: ");

      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
      }
      Serial.println();
      Serial.println("**********");
    }

    if (rxValue == "1") {
      Serial.println("Turn On LED");
      digitalWrite(BUILTIN_LED, HIGH);
    } else if (rxValue == "0") {
      Serial.println("Turn Off LED");
      digitalWrite(BUILTIN_LED, LOW);
    } else {
      Serial.println("Not recognized");
    }
  }
};

void setup() {
  dht.begin();
  Serial.begin(9600);
  pinMode(BUILTIN_LED, OUTPUT);

  BLEDevice::init("ESP32 BLE TxRx");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic (
    TX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
  pTxCharacteristic->addDescriptor(new BLE2902());
  //receive
    pRxCharacteristic = pService->createCharacteristic (
    RX_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
  pRxCharacteristic->addDescriptor(new BLE2902());
  pRxCharacteristic->setValue("0");
  pRxCharacteristic->setCallbacks(new BLECallback());
  pService->start();

  pServer->getAdvertising()->setMinPreferred(0x06);
  pServer->getAdvertising()->setMaxPreferred(0x12);
  pServer->getAdvertising()->start();
  Serial.println("Characteristic defined");
}

void loop() {

  if (millis() - lastTransmit > 5000) {
    updateDhtData();
    String sensorData = String(temperature) + ";" + String(humidity);
    pTxCharacteristic->setValue(sensorData.c_str());
    pTxCharacteristic->notify();
    lastTransmit = millis();
  }
}

void updateDhtData() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}