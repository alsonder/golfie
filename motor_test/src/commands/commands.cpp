#include "commands/commands.h"
#include "bleServer/bleServer.h"
#include <Arduino.h>


int motorA1 = 25; // IN1 on the L298N
int motorA2 = 26; // IN2 on the L298N
int motorAEnable = 27; // ENA on the L298N

// Motor B
int motorB1 = 32; // IN3 on the L298N
int motorB2 = 33; // IN4 on the L298N
int motorBEnable = 14; // ENB on the L298N

int pwmMotorA = 110; //left motor
int pwmMotorB = 130; //right motor

int currentState = STATE_OFF;

void moveForward() {
  Serial.println("Moving Forward");
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorAEnable, pwmMotorA); 
  analogWrite(motorBEnable, pwmMotorB); 
}

void moveBackward() {
  Serial.println("Moving Backward");
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH); 
  analogWrite(motorAEnable, pwmMotorA); 
  analogWrite(motorBEnable, pwmMotorB); 
  } 
void turnRight() {
  Serial.println("Turning right");
  // Make Motor A (right motor) move forward
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  // Make Motor B (left motor) move backward
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  analogWrite(motorAEnable, pwmMotorA); 
  analogWrite(motorBEnable, pwmMotorB); 
}
void turnLeft() {
  Serial.println("Turning left");
  // Make Motor A (right motor) move backward
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  // Make Motor B (left motor) move forward
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorAEnable, pwmMotorA); 
  analogWrite(motorBEnable, pwmMotorB); 
}
void stopMovement() {
  Serial.println("Stopping");
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW); 
  analogWrite(motorAEnable, 0); // Full speed
  analogWrite(motorBEnable, 0);
}

void updateFans(){
  switch (currentState) {
    case STATE_OFF:
      Serial.println("off\n");
      digitalWrite(SUCTION_FAN, HIGH);  // Turn off Fan 1
      digitalWrite(BLOWER_FAN, HIGH);  // Turn off Fan 2
      break;
    case STATE_FAN_1:
      Serial.println("suck on\n");
      digitalWrite(SUCTION_FAN, LOW);   // Turn on Fan 1
      digitalWrite(BLOWER_FAN, HIGH);  // Ensure Fan 2 is off
      break;
    case STATE_FAN_2:
      Serial.println("blow on\n");

      digitalWrite(SUCTION_FAN, HIGH);  // Ensure Fan 1 is off
      digitalWrite(BLOWER_FAN, LOW);   // Turn on Fan 2
      break;
  }
}

void setPwmMotorA(int pwmValue) {
    pwmMotorA = pwmValue;
    // Update the BLE characteristic with the new value
    std::string value = std::to_string(pwmValue);
    pwmMotorACharacteristic->setValue(value);
    pwmMotorACharacteristic->notify(); // Send notification to connected clients
}

void setPwmMotorB(int pwmValue) {
    pwmMotorB = pwmValue;
    std::string value = std:: to_string(pwmValue);
    pwmMotorBCharacteristic -> setValue(value);
    pwmMotorBCharacteristic->notify();
}
