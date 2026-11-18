#include <Arduino.h>

#include "Watchdog.h"
#include "Data.h"
#include "CAN_Com.h"

// Watchdog timeout limits.
// NOTE: the Teensys must be sending telemetry at 100 Hz before enabling these short timeouts.
// While testing at 1 Hz, raise CAN_TIMEOUT_MS to 2000 or more.
const uint32_t COMMAND_TIMEOUT_MS = 500;       // Receiving current cmds from Simulink
const uint32_t CAN_TIMEOUT_MS = 500;           // Receiving telemetry from Teensys
const uint32_t STARTUP_GRACE_PERIOD_MS = 5000; // Time for Simulink to connect and Teensys to start

// Command communication tracking of simulink command times
uint32_t lastCommandTime = 0;
bool commandReceived = false;

// CAN communication tracking of packet times
uint32_t lastCANTelemetryTime[4] = {};
bool canTelemetryReceived[4] = {};

// Startup timer
uint32_t startupTime = 0;

// Safety state
bool safeStateActive = false;


// Initialize safety flags and watchdog timers.
// Call at the END of setup() so the grace period starts after WiFi/IMU init (which takes over a second).
void initializeSafety() {

    startupTime = millis();

    lastCommandTime = startupTime;
    commandReceived = false;

    safeStateActive = false;

    for (int i = 0; i < 4; i++) {
        lastCANTelemetryTime[i] = startupTime;
        canTelemetryReceived[i] = false;
    }
}


// Record successful Simulink command reception
void recordValidCommand() {

    lastCommandTime = millis();
    commandReceived = true;
}


// Record successful CAN telemetry reception
void recordCANTelemetry(uint8_t motorIndex) {

    if (motorIndex >= 4) {
        return;
    }

    lastCANTelemetryTime[motorIndex] = millis();
    canTelemetryReceived[motorIndex] = true;
}


// Detect when Simulink stops sending commands
bool checkCommandTimeout() {

    uint32_t currentTime = millis();

    // Allow startup grace period before first command
    if (!commandReceived) {
        return (currentTime - startupTime) > STARTUP_GRACE_PERIOD_MS;
    }

    return (currentTime - lastCommandTime) > COMMAND_TIMEOUT_MS;
}


// Detect when an ACTIVE Teensy stops sending telemetry
bool checkCANTimeout() {

    uint32_t currentTime = millis();

    for (int i = 0; i < 4; i++) {

        // Inactive motors never send telemetry, so they must not trigger a fault
        if (!ACTIVE_MOTORS[i]) {
            continue;
        }

        // Allow startup grace period before first telemetry
        if (!canTelemetryReceived[i]) {

            if ((currentTime - startupTime) > STARTUP_GRACE_PERIOD_MS) {
                return true;
            }

        }
        else if ((currentTime - lastCANTelemetryTime[i]) > CAN_TIMEOUT_MS) {
            return true;
        }
    }

    return false;
}


// Force all motor current commands to zero.
// The main loop already transmits motorCommands at 100 Hz, so no extra transmit is needed here.
void enterSafeState() {

    for (int i = 0; i < 4; i++) {
        motorCommands.current_cmd[i] = 0.0f;
    }

    if (!safeStateActive) {
        Serial.println("SAFETY FAULT: commanding all motors to 0 A");
        safeStateActive = true;
    }
}


// Clear the fault state once communication has recovered
void resetSafetyFlags() {

    if (safeStateActive) {
        Serial.println("Safety flags reset.");
    }

    safeStateActive = false;
}


// Check system communication. Call every loop pass, AFTER new commands are processed
// and BEFORE transmitCANCommands(), so a fault always overrides the command being sent.
void checkWatchdogs() {

    const bool commandFault = checkCommandTimeout();
    const bool canFault = checkCANTimeout();

    if (commandFault || canFault) {

        // Re-zero on every call while the fault persists, so a late Simulink command
        // can't push current to a Teensy that has gone silent.
        if (!safeStateActive) {
            if (commandFault) Serial.println("WARNING: Simulink command timeout!");
            if (canFault)     Serial.println("WARNING: CAN telemetry timeout!");
        }
        enterSafeState();
    }
    else {
        resetSafetyFlags();
    }
}