#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "bleServer/bleServer.h"
#include "commands/commands.h"
#include "sensors/sensors.h"



void setup() {
    Serial.begin(115200);
    BLEDevice::init("ESP_ballsucking_contraption");

    // Set BLE power levels
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9); //powerlevels on the bluetooth transmission P9 = max range, but also max batteryusage
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
    BLEDevice::setPower(ESP_PWR_LVL_P3);

    // Create and configure BLE server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a characteristic for motor control commands
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
                      );
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());

    // Create characteristics for PWM control of motors
BLECharacteristic *pwmMotorACharacteristic = pService->createCharacteristic(
                                                 PWM_MOTOR_A_CHARACTERISTIC_UUID,
                                                 BLECharacteristic::PROPERTY_READ |
                                                 BLECharacteristic::PROPERTY_NOTIFY |
                                                 BLECharacteristic::PROPERTY_WRITE
                                             );
pwmMotorACharacteristic->addDescriptor(new BLE2902()); // Enable notifications

BLECharacteristic *pwmMotorBCharacteristic = pService->createCharacteristic(
                                                 PWM_MOTOR_B_CHARACTERISTIC_UUID,
                                                 BLECharacteristic::PROPERTY_READ |
                                                 BLECharacteristic::PROPERTY_NOTIFY |
                                                 BLECharacteristic::PROPERTY_WRITE
                                             );
pwmMotorBCharacteristic->addDescriptor(new BLE2902()); // Enable notifications
pwmMotorACharacteristic->setValue(std::to_string(pwmMotorA));
pwmMotorBCharacteristic->setValue(std::to_string(pwmMotorB));


    // Start the service
    pService->start();

    // Configure and start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("Waiting for a client connection to notify...");

    // Initialize motor and fan pins
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


