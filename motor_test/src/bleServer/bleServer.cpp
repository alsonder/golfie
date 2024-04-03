#include "bleServer/bleServer.h"
#include "commands/commands.h"
#include <Arduino.h>


BLECharacteristic *pCharacteristic;
BLECharacteristic *pwmMotorACharacteristic;
BLECharacteristic *pwmMotorBCharacteristic;

bool isConnected = false;

unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000; // Timeout in milliseconds, adjust as needed
char lastCommand = '\0'; // Stores the last command received
bool isMoving = false;

void MyServerCallbacks::onConnect(BLEServer* pServer) {
    Serial.println("Client Connected");
    isConnected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    Serial.println("Client Disconnected");
    isConnected = false;
    BLEDevice::startAdvertising(); // Restart advertising upon disconnection
    Serial.println("Advertising started");
}

void MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (!value.empty()) {
        char cmd = value[0];
        handleBluetoothCommands(&cmd);
    }
}



MotorPWMCallbacks::MotorPWMCallbacks(int motor) : _motor(motor) {
    // constructor for pwm value setting
}

// onWrite method definition
void MotorPWMCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (!value.empty()) {
        int pwmValue = std::stoi(value);
        if (pCharacteristic == pwmMotorACharacteristic) {
            setPwmMotorA(pwmValue);
            Serial.println("pwm value set motor A");
            Serial.println(pwmValue);
        } else if (pCharacteristic == pwmMotorBCharacteristic) {
            setPwmMotorB(pwmValue);
            Serial.println("pwm value set motor B");
            Serial.println(pwmValue);
        }
    }
}


void handleBluetoothCommands(char *cmd) {
    if (cmd[0] == 'p' && strchr(cmd, ' ') != NULL) {
        // Command to adjust PWM for Motor A: format "p <newpwmvalue>"
        int pwmValue = atoi(strchr(cmd, ' ') + 1); // Convert the substring after the space to int
        setPwmMotorA(pwmValue);
        Serial.println("new pwm set motor a");
        Serial.println(pwmValue);
    } else if (cmd[0] == 'q' && strchr(cmd, ' ') != NULL) {
        // Command to adjust PWM for Motor B: format "q <newpwmvalue"
        int pwmValue = atoi(strchr(cmd, ' ') + 1); // Convert the substring after the space to int
        setPwmMotorB(pwmValue);
        Serial.println("new pwm set motor b");
        Serial.println(pwmValue);
    } else {
        // Handle movement commands
        switch (cmd[0]) {
            case 'w': // Forward
                moveForward();
                break;
            case 's': // Backward
                moveBackward();
                break;
            case 'a': // Turn Left
                turnLeft();
                break;
            case 'd': // Turn Right
                turnRight();
                break;
            case 'c': // Stop
                stopMovement();
                break;
            case 'f': // Fan Suck
                currentState = STATE_FAN_1;
                updateFans();
                break;
            case 'g': // Fan Blow
                currentState = STATE_FAN_2;
                updateFans();
                break;
            case 'r': // Fans Off
                currentState = STATE_OFF;
                updateFans();
                break;
            
        }
    }
}
