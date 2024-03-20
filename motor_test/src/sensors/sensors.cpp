#include "sensors/sensors.h"
#include <Arduino.h>

const int voltageSensorPin = 35;
const int currentSensorPin = 34;

const float adcResolution = 4095.0; // 12-bit ADC
const float referenceVoltage = 3.3; // Reference voltage of ESP32 ADC
const float sensorSensitivity = 0.185; // Sensitivity in V/A (185 mV/A for ACS712 ±5A model)
const float zeroCurrentVoltage = referenceVoltage / 2; 
const float voltageDividerRatio = 5.004; // Effective voltage divider ratio


float convertToVoltage(int reading) {
  float adcResolution = 4095.0; // 12-bit ADC
  float referenceVoltage = 3.3; // Reference voltage of ESP32 ADC
  float maxInputVoltage = 25.0; // Maximum voltage the sensor can measure
  float sensorOutputAtMaxVoltage = 3.3; // Sensor's output voltage at its maximum measurable voltage
  // Calculate the voltage division ratio
  float divisionRatio = maxInputVoltage / sensorOutputAtMaxVoltage;
  // Convert reading to voltage
  float voltageReading = (reading / adcResolution) * referenceVoltage;
  float actualVoltage = voltageReading * divisionRatio;
  return actualVoltage;
}
float readCurrent() {
  int adcValue = analogRead(currentSensorPin);
  float voltageReading = (adcValue / adcResolution) * referenceVoltage;
  float current = (voltageReading - zeroCurrentVoltage) / sensorSensitivity;
  return current;
}
float readVoltage(int pin) {
  int sensorValue = analogRead(pin);
  float voltage = (sensorValue / adcResolution) * referenceVoltage;
  return voltage;
}

// Function to calculate actual input voltage
float calculateInputVoltage(float readVoltage, float dividerRatio) {
  return readVoltage * dividerRatio;
} 