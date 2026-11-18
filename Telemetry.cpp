#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "Telemetry.h"
#include "Data.h"
#include "IMU.h"
#include "Wifi_Access_Point.h"

// 1 = fixed-size 108-byte binary packets (for Simulink TCP/IP Receive + Byte Unpack)
// 0 = CSV text line, 27 fields ending in '\n'
#define TELEMETRY_BINARY 1

// CSV packet buffer (used when TELEMETRY_BINARY is 0)
char telemetryBuffer[512] = {};

// Check whether a valid telemetry packet has been created
bool telemetryPacketReady = false;

// Telemetry transmission rate
const uint32_t TELEMETRY_INTERVAL_US = 10000; // 100 Hz

uint32_t previousTelemetryTime = 0;


#if TELEMETRY_BINARY

// Binary packet layout, little-endian, 108 bytes total:
//   byte 0-1     sync header 0xAA 0x55
//   byte 2-5     uint32  IMU timestamp [us]
//   byte 6-105   25 x single, in this order:
//                  q current [A] x4, d current [A] x4, velocity [rad/s] x4, angle [rad] x4,
//                  gyro xyz [rad/s], accel xyz [m/s^2], mag xyz [uT]
//   byte 106     int8    IMU temperature [C]
//   byte 107     uint8   checksum = sum of bytes 0..106, modulo 256
const size_t TELEMETRY_PACKET_SIZE = 108;

static uint8_t telemetryPacket[TELEMETRY_PACKET_SIZE];
static size_t packetOffset = 0;

// Copy bytes with memcpy (not struct pointers) so there are no unaligned float accesses
static void putBytes(const void* src, size_t n) {
    memcpy(&telemetryPacket[packetOffset], src, n);
    packetOffset += n;
}

static void putFloat(float v) {
    putBytes(&v, sizeof(float));
}

#endif


// Initialize telemetry structures
void initializeTelemetry() {

    memset(telemetryBuffer, 0, sizeof(telemetryBuffer));

    telemetryPacketReady = false;

    previousTelemetryTime = micros();
}


void packTelemetry() {

#if TELEMETRY_BINARY

    packetOffset = 0;

    const uint8_t sync[2] = { 0xAA, 0x55 };
    putBytes(sync, 2);

    const uint32_t timestamp = imuTelemetry.timestamp_us;
    putBytes(&timestamp, sizeof(timestamp));

    for (int i = 0; i < 4; i++) putFloat(motorTelemetry.current_q[i]);
    for (int i = 0; i < 4; i++) putFloat(motorTelemetry.current_d[i]);
    for (int i = 0; i < 4; i++) putFloat(motorTelemetry.velocity[i]);
    for (int i = 0; i < 4; i++) putFloat(motorTelemetry.angle[i]);

    putFloat(imuTelemetry.gyro_x);
    putFloat(imuTelemetry.gyro_y);
    putFloat(imuTelemetry.gyro_z);

    putFloat(imuTelemetry.accel_x);
    putFloat(imuTelemetry.accel_y);
    putFloat(imuTelemetry.accel_z);

    putFloat(imuTelemetry.mag_x);
    putFloat(imuTelemetry.mag_y);
    putFloat(imuTelemetry.mag_z);

    const int8_t temperature = imuTelemetry.temperature;
    putBytes(&temperature, sizeof(temperature));

    uint8_t checksum = 0;
    for (size_t i = 0; i < packetOffset; i++) {
        checksum += telemetryPacket[i];
    }
    putBytes(&checksum, sizeof(checksum));

    if (packetOffset == TELEMETRY_PACKET_SIZE) {
        telemetryPacketReady = true;
    }
    else {
        telemetryPacketReady = false;
        Serial.println("Telemetry packet size mismatch!");
    }

#else

    // CSV line with 27 fields:
    // timestamp, q[4], d[4], velocity[4], angle[4], gyro[3], accel[3], mag[3], temperature
    int length = snprintf(
        telemetryBuffer,
        sizeof(telemetryBuffer),

        "%lu,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%d\n",

        (unsigned long)imuTelemetry.timestamp_us,

        motorTelemetry.current_q[0], motorTelemetry.current_q[1],
        motorTelemetry.current_q[2], motorTelemetry.current_q[3],

        motorTelemetry.current_d[0], motorTelemetry.current_d[1],
        motorTelemetry.current_d[2], motorTelemetry.current_d[3],

        motorTelemetry.velocity[0], motorTelemetry.velocity[1],
        motorTelemetry.velocity[2], motorTelemetry.velocity[3],

        motorTelemetry.angle[0], motorTelemetry.angle[1],
        motorTelemetry.angle[2], motorTelemetry.angle[3],

        imuTelemetry.gyro_x, imuTelemetry.gyro_y, imuTelemetry.gyro_z,

        imuTelemetry.accel_x, imuTelemetry.accel_y, imuTelemetry.accel_z,

        imuTelemetry.mag_x, imuTelemetry.mag_y, imuTelemetry.mag_z,

        (int)imuTelemetry.temperature);

    if (length > 0 && (size_t)length < sizeof(telemetryBuffer)) {
        telemetryPacketReady = true;
    }
    else {
        telemetryPacketReady = false;
        Serial.println("Telemetry packet formatting failed!");
    }

#endif
}


// Send the completed telemetry packet to Simulink
void sendTelemetry() {

    if (!telemetryPacketReady) {
        return;
    }

#if TELEMETRY_BINARY
    sendWifiBytes(telemetryPacket, TELEMETRY_PACKET_SIZE);
#else
    sendWifiTelemetry(telemetryBuffer);
#endif

    telemetryPacketReady = false;
}


// Read the IMU, combine with the latest motor data, and send at 100 Hz.
// Call every loop pass; this function is the only rate limiter.
void updateTelemetry() {

    uint32_t currentTime = micros();

    if ((uint32_t)(currentTime - previousTelemetryTime) < TELEMETRY_INTERVAL_US) {
        return;
    }

    // Advance by a fixed step so the rate doesn't drift; resync if we fell far behind
    previousTelemetryTime += TELEMETRY_INTERVAL_US;
    if ((uint32_t)(currentTime - previousTelemetryTime) > 5 * TELEMETRY_INTERVAL_US) {
        previousTelemetryTime = currentTime;
    }

    // Sample the IMU right before packing so the IMU data and packet are the same age
    readIMU();
    updateIMUData();

    packTelemetry();

    sendTelemetry();
}


/*
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "Telemetry.h"
#include "Data.h"
#include "IMU.h"
#include "Wifi_Access_Point.h"

// Telemetry packet buffer
char telemetryBuffer[512] = {};

// Check whether a valid telemetry packet has been created
bool telemetryPacketReady = false;

// Telemetry transmission rate
const uint32_t TELEMETRY_INTERVAL_US = 10000; // 100 Hz

uint32_t previousTelemetryTime = 0;


// Initialize telemetry structures
void initializeTelemetry() {

    memset(telemetryBuffer, 0, sizeof(telemetryBuffer));

    telemetryPacketReady = false;

    previousTelemetryTime = micros();
}


// Format telemetry measurements into one CSV line with 27 fields:
// timestamp, q[4], d[4], velocity[4], angle[4], gyro[3], accel[3], mag[3], temperature
void packTelemetry() {

    int length = snprintf(
        telemetryBuffer,
        sizeof(telemetryBuffer),

        "%lu,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%d\n",

        (unsigned long)imuTelemetry.timestamp_us,

        motorTelemetry.current_q[0], motorTelemetry.current_q[1],
        motorTelemetry.current_q[2], motorTelemetry.current_q[3],

        motorTelemetry.current_d[0], motorTelemetry.current_d[1],
        motorTelemetry.current_d[2], motorTelemetry.current_d[3],

        motorTelemetry.velocity[0], motorTelemetry.velocity[1],
        motorTelemetry.velocity[2], motorTelemetry.velocity[3],

        motorTelemetry.angle[0], motorTelemetry.angle[1],
        motorTelemetry.angle[2], motorTelemetry.angle[3],

        imuTelemetry.gyro_x, imuTelemetry.gyro_y, imuTelemetry.gyro_z,

        imuTelemetry.accel_x, imuTelemetry.accel_y, imuTelemetry.accel_z,

        imuTelemetry.mag_x, imuTelemetry.mag_y, imuTelemetry.mag_z,

        (int)imuTelemetry.temperature);

    if (length > 0 && (size_t)length < sizeof(telemetryBuffer)) {
        telemetryPacketReady = true;
    }
    else {
        telemetryPacketReady = false;
        Serial.println("Telemetry packet formatting failed!");
    }
}


// Send the completed telemetry packet to Simulink
void sendTelemetry() {

    if (!telemetryPacketReady) {
        return;
    }

    sendWifiTelemetry(telemetryBuffer);

    telemetryPacketReady = false;
}


// Read the IMU, combine with the latest motor data, and send at 100 Hz.
// Call every loop pass; this function is the only rate limiter.
void updateTelemetry() {

    uint32_t currentTime = micros();

    if ((uint32_t)(currentTime - previousTelemetryTime) < TELEMETRY_INTERVAL_US) {
        return;
    }

    // Advance by a fixed step so the rate doesn't drift; resync if we fell far behind
    previousTelemetryTime += TELEMETRY_INTERVAL_US;
    if ((uint32_t)(currentTime - previousTelemetryTime) > 5 * TELEMETRY_INTERVAL_US) {
        previousTelemetryTime = currentTime;
    }

    // Sample the IMU right before packing so the IMU data and packet are the same age
    readIMU();
    updateIMUData();

    packTelemetry();

    sendTelemetry();
}
*/