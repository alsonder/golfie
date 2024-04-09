#ifndef BLESERVER_H
#define BLESERVER_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define PWM_MOTOR_A_CHARACTERISTIC_UUID "d973f2e0-b19e-11e2-9e96-0800200c9a66"
#define PWM_MOTOR_B_CHARACTERISTIC_UUID "d973f2e1-b19e-11e2-9e96-0800200c9a66"

extern bool isConnected;
extern BLECharacteristic *pCharacteristic;
extern BLECharacteristic *pwmMotorACharacteristic;
extern BLECharacteristic *pwmMotorBCharacteristic;
extern unsigned long lastCommandTime;
extern const unsigned long commandTimeout; // Timeout in milliseconds
extern char lastCommand; // Stores the last command received
extern bool isMoving;


//BLE server callback classes
class MyServerCallbacks : public BLEServerCallbacks {
public:
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
};

class MyCallbacks : public BLECharacteristicCallbacks {
public:
    void onWrite(BLECharacteristic *pCharacteristic) override;
}; 

class MotorPWMCallbacks : public BLECharacteristicCallbacks {
public:
    MotorPWMCallbacks(int motor);
    void onWrite(BLECharacteristic *pCharacteristic) override;
private:
    int _motor;
};

void handleBluetoothCommands(char *cmd);



#endif