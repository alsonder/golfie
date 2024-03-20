#include "bleServer/bleServer.h"
#include "movement/movement.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h> 

BLECharacteristic *pCharacteristic;

bool isConnected = false;

unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000; // Timeout in milliseconds, adjust as needed
char lastCommand = '\0'; // Stores the last command received
bool isMoving = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
      Serial.println("Client Connected");
      isConnected = true;
    }

    void onDisconnect(BLEServer* pServer) override {
      Serial.println("Client Disconnected");
      isConnected = false;
      BLEDevice::startAdvertising(); // Restart advertising upon disconnection
      Serial.println("Advertising started");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
      std::string value = pCharacteristic->getValue();
      if (!value.empty()) {
        char cmd = value[0];
        handleBluetoothCommands(cmd);
      }
    }
};

void handleBluetoothCommands(char cmd){
  lastCommandTime = millis();
  lastCommand = cmd;
  isMoving = true; 
    switch (cmd){
      case 'w': 
        moveForward(); 
        Serial.println(cmd);
        break;
      case 's': 
        moveBackward(); 
        Serial.println(cmd);
        break;
      case 'a': 
        turnLeft(); 
        Serial.println(cmd);
        break;
      case 'd': 
        turnRight(); 
        Serial.println(cmd);
        break; 
      case 'c': 
        stopMovement(); 
        Serial.println(cmd);
        break; 
      case 'f': 
        currentState = STATE_FAN_1; 
        updateFans();
        break;
      case 'g': 
        currentState = STATE_FAN_2; 
        updateFans();
        break;
      case 'r': 
        currentState = STATE_OFF; 
        updateFans();
        break;
      default: break;
    } 
}