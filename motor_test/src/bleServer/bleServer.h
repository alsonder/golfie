#ifndef BLESERVER_H
#define BLESERVER_H

#include <BLEServer.h>

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

extern bool isConnected;
extern BLECharacteristic *pCharacteristic;
extern unsigned long lastCommandTime;
extern const unsigned long commandTimeout; // Timeout in milliseconds, adjust as needed
extern char lastCommand; // Stores the last command received
extern bool isMoving;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer);

    void onDisconnect(BLEServer* pServer);
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic);
    
};
void handleBluetoothCommands(char cmd);



#endif