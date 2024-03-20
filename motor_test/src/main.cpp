#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h> 
#include "bleServer/bleServer.h"
#include "commands/commands.h"
#include "sensors/sensors.h"

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

unsigned long previousMillis = 0; 
const long interval = 2000; //2 second voltage and current readings

bool isMoving = false; // Flag to track movement state

void setup() {
  
  Serial.begin(115200);
  BLEDevice::init("ESP_ballsucking_contraption");
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9); // Default power
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9); // Advertising power
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9); // Scan power

  BLEDevice::setPower(ESP_PWR_LVL_P3);//sets the db 
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->setCallbacks(new MyCallbacks()); 
   
  pCharacteristic->setValue("Hello World");
  pCharacteristic->addDescriptor(new BLE2902()); // Enable notifications

  pService->start();

  // New Advertising Setup
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Functions that might help with faster connections
  pAdvertising->setMinPreferred(0x12);
  
  // Start Advertising
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAEnable, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBEnable, OUTPUT);
 
  pinMode(SUCTION_FAN, OUTPUT);
  pinMode(BLOWER_FAN, OUTPUT);
  digitalWrite(SUCTION_FAN, HIGH);
  digitalWrite(BLOWER_FAN, HIGH);
  pinMode(voltageSensorPin, INPUT);
  pinMode(currentSensorPin, INPUT);
}
void loop() {

  if (isMoving && (millis() - lastCommandTime > commandTimeout)) {
    stopMovement(); // Make sure this function stops all motors
    isMoving = false;
    Serial.println("Stopping due to timeout");
  }


}


