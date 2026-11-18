#include <string.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

#include "Wifi_Access_Point.h"

// Implementation of functions for wifi access point between esp32 and simulink

// WiFi configuration
const char* ssid = "Hampter";       // Wifi name
const char* password = "Whirligig"; // Wifi Password

// TCP server
WiFiServer server(80);

// Connected Simulink client
WiFiClient client;

// Incoming command buffer to hold Simulink data
char rxBuffer[128];
size_t rxIndex = 0;


// Initialize WiFi access point and TCP server
void setupWifi() {

    Serial.println();
    Serial.println("Configuring access point...");

    if (!WiFi.softAP(ssid, password)) {
        Serial.println("WiFi failed!");
        return;
    }

    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.softAPIP());

    server.begin();

    Serial.println("TCP server started on port 80");
}


// Receive commands from Simulink
bool receiveWifiCommand(char* buffer, size_t bufferSize) {

    // Accept a new client if one is not connected
    if (!client || !client.connected()) {

        if (client) {
            client.stop();
            Serial.println("Simulink disconnected");
        }

        client = server.available();

        if (client) {
            client.setNoDelay(true);   // send small telemetry packets immediately (disable Nagle)
            Serial.println("Simulink connected!");
            rxIndex = 0;
        }

        return false;
    }

    // Read incoming data without blocking
    while (client.available()) {

        char c = client.read();

        // Newline indicates end of command packet
        if (c == '\n') {

            // Remove carriage return if present
            if (rxIndex > 0 && rxBuffer[rxIndex - 1] == '\r') {
                rxIndex--;
            }

            // Output buffer needs room for the terminating null
            if (rxIndex >= bufferSize) {
                rxIndex = 0;
                return false;
            }

            memcpy(buffer, rxBuffer, rxIndex);
            buffer[rxIndex] = '\0';
            rxIndex = 0;

            return true;
        }

        // Store received character
        if (rxIndex < sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = c;
        }
        else {
            // Discard oversized command
            rxIndex = 0;
        }
    }

    return false;
}


// Send telemetry to Simulink. The packet already ends in '\n', so write it as-is in one call
// (println would add an extra "\r\n" and can split into two TCP writes).
void sendWifiTelemetry(const char* data) {

    if (client && client.connected()) {
        client.write((const uint8_t*)data, strlen(data));
    }
}


// Send a raw binary packet to Simulink (binary data can contain zero bytes, so no strlen)
void sendWifiBytes(const uint8_t* data, size_t length) {

    if (client && client.connected()) {
        client.write(data, length);
    }
}


// Check whether a computer is connected to the access point
bool checkWiFiConnection() {

    return WiFi.softAPgetStationNum() > 0;
}

