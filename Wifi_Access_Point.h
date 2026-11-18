#ifndef WIFI_ACCESS_POINT_H
#define WIFI_ACCESS_POINT_H

#include <Arduino.h>

// Declares funcitons for wifi access point between esp32 and simulink

// Initialize WiFi access point and TCP server
void setupWifi();

// Check for incoming Simulink commands
bool receiveWifiCommand(char* buffer, size_t bufferSize);

// Send CSV text telemetry back to Simulink
void sendWifiTelemetry(const char* data);

// Send a raw binary telemetry packet back to Simulink
void sendWifiBytes(const uint8_t* data, size_t length);

// Check WiFi connection status
bool checkWiFiConnection();

#endif
