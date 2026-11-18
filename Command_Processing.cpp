#include <Arduino.h>
#include <stdlib.h>
#include <math.h>

#include "Command_Processing.h"
#include "Data.h"

// Set to 1 for command debug prints (blocks the loop at 100 Hz, so keep off normally)
#define CMD_DEBUG_PRINTS 0

// Temporary storage for incoming commands from Simulink that have not been validated
float receivedCommands[4];

// Check if command received is in valid format
bool commandFormatValid = false;

// Check if at least one valid command has been accepted (format and current limit checks are passed)
bool validCommandReceived = false;

// Maximum magnitude of allowed current to be commanded
const float MAX_CURRENT = 1.1f; // [A]


// Initialize command variables and processing state
void initializeCommandProcessing() {

    for (int i = 0; i < 4; i++) {
        receivedCommands[i] = 0.0f;
        motorCommands.current_cmd[i] = 0.0f;
    }

    validCommandReceived = false;
    commandFormatValid = false;
}

// Convert received TCP data from Simulink into four floating-point current commands. Store in receivedCommands
void unpackCommands() {

    commandFormatValid = false;

    char* ptr = receivedCommandBuffer;

    // Parse into a temp array so a bad packet never half-overwrites receivedCommands
    float parsed[4];

    for (int i = 0; i < 4; i++) {

        char* endPtr;
        parsed[i] = strtof(ptr, &endPtr);

        if (ptr == endPtr) {
            Serial.println("Command from Simulink is not a number");
            return;
        }

        ptr = endPtr;

        while (*ptr == ' ' || *ptr == '\t' || *ptr == '\r') {
            ptr++;
        }

        if (i < 3) {
            if (*ptr != ',') {
                Serial.println("Cannot find comma delimination in Simulink Command");
                return;
            }
            ptr++;
        }
    }

    while (*ptr == ' ' || *ptr == '\t' || *ptr == '\r') {
        ptr++;
    }

    if (*ptr != '\0') {
        Serial.println("Cannot find command terminator in Simulink Command");
        return;
    }

    for (int i = 0; i < 4; i++) {
        receivedCommands[i] = parsed[i];
    }

    commandFormatValid = true;
}

// Check that the received command packet is valid
bool validateCommands() {

    if (!commandFormatValid) {
        Serial.println("Invalid command format from Simulink");
        return false;
    }

    for (int i = 0; i < 4; i++) {
        if (!isfinite(receivedCommands[i])) {
            Serial.println("Command contains an invalid numerical value");
            return false;
        }
    }

    return true;
}

// Ensure all four current commands are within allowable limits
bool checkCurrentLimits() {

    for (int i = 0; i < 4; i++) {
        if (fabsf(receivedCommands[i]) > MAX_CURRENT) {
            Serial.printf("Motor %d current command exceeds allowable limit\n", i + 1);
            return false;
        }
    }

    return true;
}

// Update the shared motor command structure after validation
bool processCommands() {

    if (!validateCommands()) {
        return false;
    }

    if (!checkCurrentLimits()) {
        return false;
    }

    for (int j = 0; j < 4; j++) {
        motorCommands.current_cmd[j] = receivedCommands[j];
#if CMD_DEBUG_PRINTS
        Serial.printf("Motor %d cmd %.4f validated\n", j + 1, receivedCommands[j]);
#endif
    // poss add delay
    }

    validCommandReceived = true;

    return true;   // was missing: falling off the end of a bool function is undefined behavior
}

// Track command sequence numbers to detect missing or repeated packets
void updateCommandSequence() {

}
