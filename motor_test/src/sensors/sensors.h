#ifndef SENSORS_H
#define SENSORS_H

extern "C"{
    const int voltageSensorPin;
    const int currentSensorPin;

    const float adcResolution; // 12-bit ADC
    const float referenceVoltage; // Reference voltage of ESP32 ADC
    const float sensorSensitivity; // Sensitivity in V/A (185 mV/A for ACS712 ±5A model)
    const float zeroCurrentVoltage = referenceVoltage / 2; 
    const float voltageDividerRatio; // Effective voltage divider ratio

}

#endif