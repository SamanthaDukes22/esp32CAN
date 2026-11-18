#include <Arduino.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/twai.h"

#include "CAN_Com.h"
#include "Data.h"
#include "Watchdog.h"

#define TX_GPIO_NUM   GPIO_NUM_25 // TX on ESP32 (to transceiver TXD/CTX)
#define RX_GPIO_NUM   GPIO_NUM_26 // RX on ESP32 (to transceiver RXD/CRX)

// Set to 1 to see per-frame debug prints (slows the loop a lot)
#define CAN_DEBUG_PRINTS 0

// Received CAN message information (filled by receiveCANTelemetry)
uint8_t canBuffer[8];
int receivedPacketSize = 0;
long receivedCANID = 0;

bool processCANMessage();

// If the bus has gone bus-off or the driver stopped, bring it back
static void recoverCANIfNeeded() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) return;

  if (status.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();   // driver moves to STOPPED when recovery finishes
  } else if (status.state == TWAI_STATE_STOPPED) {
    twai_start();
  }
}

// Initialize ESP32 TWAI controller (CAN) at 500 kbit/s, standard frames
void setupCAN() {

  Serial.println("Initiating ESP32 for CAN communication");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 8;
  g_config.rx_queue_len = 64;   // ~800 frames/s at 100 Hz from 4 Teensys; headroom for loop jitter

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  // 500 kbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("Failed to install TWAI driver!");
    while (1) { delay(1000); }
  }

  if (twai_start() != ESP_OK) {
    Serial.println("Failed to start TWAI driver!");
    while (1) { delay(1000); }
  }

  Serial.println("ESP32 ready for CAN communication");
}


// Send motor commands to the four Teensys from ESP32
void transmitCANCommands() {

  recoverCANIfNeeded();

  // CAN command IDs for each motor (must be 11-bit, max 0x7FF)
  const uint16_t motorCommandIDs[4] = {
      CAN_ID_MOTOR1_COMMAND,
      CAN_ID_MOTOR2_COMMAND,
      CAN_ID_MOTOR3_COMMAND,
      CAN_ID_MOTOR4_COMMAND
  };

  for (int i = 0; i < 4; i++) {

    if (!ACTIVE_MOTORS[i]) {
      continue;
    }

    float command = motorCommands.current_cmd[i];

    twai_message_t message = {};            // zero-init all flags
    message.identifier = motorCommandIDs[i];
    message.extd = 0;                       // 11-bit standard ID
    message.rtr = 0;
    message.data_length_code = sizeof(float);
    memcpy(message.data, &command, sizeof(float));

    // Don't block: if the queue is full the bus isn't draining, and stalling won't help
    esp_err_t err = twai_transmit(&message, 0);

#if CAN_DEBUG_PRINTS
    Serial.printf("Motor %d cmd %s\n", i + 1, err == ESP_OK ? "queued" : "FAILED to queue");
#else
    (void)err;
#endif
  }
}


// Receive incoming CAN frames from the Teensys (call every loop pass)
void receiveCANTelemetry() {

  twai_message_t message;

  // Drain everything currently waiting in the RX queue (0 ticks = don't block)
  while (twai_receive(&message, 0) == ESP_OK) {

    // Only 11-bit data frames are expected from the Teensys
    if (message.extd || message.rtr) {
      continue;
    }

    receivedCANID = message.identifier;
    receivedPacketSize = message.data_length_code;

    if (receivedPacketSize != 8) {
      continue;
    }

    memcpy(canBuffer, message.data, 8);

    processCANMessage();
  }
}


// Identify the sending Teensy and write its CAN data
bool processCANMessage() {

  float data1;
  float data2;

  memcpy(&data1, &canBuffer[0], sizeof(float));
  memcpy(&data2, &canBuffer[4], sizeof(float));

  switch (receivedCANID) {

    case CAN_ID_MOTOR1_CURRENT_TELEMETRY:
      motorTelemetry.current_q[0] = data1;
      motorTelemetry.current_d[0] = data2;
      recordCANTelemetry(0);
      return true;

    case CAN_ID_MOTOR1_DYNAMIC_TELEMETRY:
      motorTelemetry.velocity[0] = data1;
      motorTelemetry.angle[0] = data2;
      recordCANTelemetry(0);
      return true;

    case CAN_ID_MOTOR2_CURRENT_TELEMETRY:
      motorTelemetry.current_q[1] = data1;
      motorTelemetry.current_d[1] = data2;
      recordCANTelemetry(1);
      return true;

    case CAN_ID_MOTOR2_DYNAMIC_TELEMETRY:
      motorTelemetry.velocity[1] = data1;
      motorTelemetry.angle[1] = data2;
      recordCANTelemetry(1);
      return true;

    case CAN_ID_MOTOR3_CURRENT_TELEMETRY:
      motorTelemetry.current_q[2] = data1;
      motorTelemetry.current_d[2] = data2;
      recordCANTelemetry(2);
      return true;

    case CAN_ID_MOTOR3_DYNAMIC_TELEMETRY:
      motorTelemetry.velocity[2] = data1;
      motorTelemetry.angle[2] = data2;
      recordCANTelemetry(2);
      return true;

    case CAN_ID_MOTOR4_CURRENT_TELEMETRY:
      motorTelemetry.current_q[3] = data1;
      motorTelemetry.current_d[3] = data2;
      recordCANTelemetry(3);
      return true;

    case CAN_ID_MOTOR4_DYNAMIC_TELEMETRY:
      motorTelemetry.velocity[3] = data1;
      motorTelemetry.angle[3] = data2;
      recordCANTelemetry(3);
      return true;

    default:
      // Prints the ID so a mismatch with the Teensy IDs is easy to spot
      Serial.printf("Unknown CAN id 0x%03lX\n", (unsigned long)receivedCANID);
      return false;
  }
}


// Print CAN controller health (call about once per second while debugging).
// Returns false if the bus is not in the running state.
bool checkCANStatus() {

  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) {
    Serial.println("CAN: could not read status");
    return false;
  }

  Serial.printf("CAN state=%d txq=%lu rxq=%lu txErr=%lu rxErr=%lu txFailed=%lu rxMissed=%lu rxOverrun=%lu arbLost=%lu busErr=%lu\n",
                (int)s.state,
                (unsigned long)s.msgs_to_tx, (unsigned long)s.msgs_to_rx,
                (unsigned long)s.tx_error_counter, (unsigned long)s.rx_error_counter,
                (unsigned long)s.tx_failed_count, (unsigned long)s.rx_missed_count,
                (unsigned long)s.rx_overrun_count, (unsigned long)s.arb_lost_count,
                (unsigned long)s.bus_error_count);

  return s.state == TWAI_STATE_RUNNING;
}


/*
#include <CAN.h>
#include <string.h>

#include "CAN_Com.h"
#include "Data.h"
#include "Watchdog.h"


#define TX_GPIO_NUM   25 // TX on ESP32
#define RX_GPIO_NUM   26 // RX on ESP32

// CAN docuementation: https://github.com/sandeepmistry/arduino-CAN/blob/master/API.md

// CAN receive buffer
uint8_t canBuffer[8];

uint8_t bytesRead = 0;

// Received CAN message information
int receivedPacketSize = 0;
long receivedCANID = 0;

// Initialize ESP32 TWAI controller and CAN configuration
void setupCAN()	{

  // MotorA CAN Setup
  Serial.println ("Initiating ESP32 for CAN communication");

  // Set the pins CAN.setPins(rx, tx);
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // Begin CAN Communication between ESP32 and Teensys
  if (!CAN.begin(CAN_BAUD_RATE)) {  // Set CAN bit rate (bps) (1000E3, 500E3, 250E3, 200E3, 125E3, 100E3, 80E3, 50E3, 40E3, 20E3, 10E3, 5E3)
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Serial.println ("ESP32 ready for CAN communication");
  
}

// Send motor commands to the four Teensys from ESP32
void transmitCANCommands()	{

  float command;
  command = motorCommands.current_cmd[0] = 0.100f;
  CAN.beginPacket(CAN_ID_MOTOR1_COMMAND);
  Serial.println("Starting CAN Packet")
  CAN.write(command);
  CAN.endPacket();
}



/*
// Send motor commands to the four Teensys from ESP32
void transmitCANCommands()	{


  // 4 CAN bus testing, set command values  
  motorCommands.current_cmd[0] = 0.100f;
  motorCommands.current_cmd[1] = 0.000f;
  motorCommands.current_cmd[2] = 0.000f;
  motorCommands.current_cmd[3] = 0.000f;

  float command;
  
  // CAN command IDs for each motor
  const uint16_t motorCommandIDs[4] = {
      CAN_ID_MOTOR1_COMMAND,
      CAN_ID_MOTOR2_COMMAND,
      CAN_ID_MOTOR3_COMMAND,
      CAN_ID_MOTOR4_COMMAND
  };  

  // Set up for sending motor cmds over CAN
  // beginPacket(id): standard packet id, 11 bits
  // write(buffer, length): data to write to packet , size of data to write
  // endPacket(): End the sequence of sending a packet. Returns 1 on success, 0 on failure.

  for (int i = 0; i < 4; i++) {

      // Skip inactive motor
      if (!ACTIVE_MOTORS[i]) {
          continue;
      }

      // Retrieve motor current command
      command = motorCommands.current_cmd[i];

      Serial.print("Attempting to send motor ");
      Serial.print(i + 1);
      Serial.println(" packet");     
      
      CAN.beginPacket(motorCommandIDs[i]); 

      // Cmd wrong size
      if (CAN.write((uint8_t*)&command, sizeof(command)) != sizeof(command)) {
          Serial.print("Failed to write motor ");
          Serial.print(i + 1);
          Serial.println(" command");
      }

      if (!CAN.endPacket()) {
          Serial.print("Motor ");
          Serial.print(i + 1);
          Serial.println(" packet failed to send");
      }
      else {
          Serial.print("Motor ");
          Serial.print(i + 1);
          Serial.println(" command has been sent");
      }
  }
}


// Receive incoming CAN frames from the Teensys
void receiveCANTelemetry() {

  // Check if a packet has been received
  receivedPacketSize = CAN.parsePacket();

  // Exit function if no packet is available
  if (receivedPacketSize == 0) {
    return;
  }

  // Returns the id (11-bit or 29 bit) of the received packet. Standard packets have an 11-bit id, extended packets have an 29-bit id
  // Should recieve 11-bit id from teensys, if not, exit function
  receivedCANID = CAN.packetId();


  // Verify expected payload size, should receive 8 bytes, if not, exit function
  if (receivedPacketSize != 8) {
    Serial.println("Packet size payload is not 8 bytes");
    while (CAN.available()) CAN.read(); // Discard bad packet
    return;
  }

  // Reset byte counter for each new CAN packet
  bytesRead = 0;

  // Read bytes from teenys and enumerate bytes read into canBuffer
  while (CAN.available() && bytesRead < 8) {
      canBuffer[bytesRead++] = CAN.read();
  }

  // Once 8 bytes have been read, ESP32 has recieved full motor telemetry package
  if (bytesRead != 8) return;

  // Move to processing the CAN message function
  processCANMessage();

}

// Identify the sending Teensy and write its CAN data
bool processCANMessage()	{

  // Temporary varibales to store CAN telemetry from each package, each package contains 2 variables(floating point #s)
  float data1;
  float data2;

  // Extract the start on each data point in package (0-3 varibale 1, 4-7 variable 2)
  memcpy(&data1, &canBuffer[0], sizeof(float));
  memcpy(&data2, &canBuffer[4], sizeof(float));

  // Based on CANID of package, identify motor # and packet type (current, dynamic, ...), separate varibales and fill motor telemetry structure
  switch (receivedCANID) {

    // Motor 1 current package
    case CAN_ID_MOTOR1_CURRENT_TELEMETRY:
        motorTelemetry.current_q[0] = data1;
        motorTelemetry.current_d[0] = data2;
        recordCANTelemetry(0); // Record successful time of communication from Motor 1
        Serial.println("Motor1 current telemetry packet has been received by ESP32");
        return true;

    // Motor 1 dynamic package
    case CAN_ID_MOTOR1_DYNAMIC_TELEMETRY:
        motorTelemetry.velocity[0] = data1;
        motorTelemetry.angle[0] = data2;
        recordCANTelemetry(0); // Record successful time of communication from Motor 1     
        Serial.println("Motor 1 dynamic telemetry packet has been received by ESP32");   
        return true;

    // Motor 2 current package
    case CAN_ID_MOTOR2_CURRENT_TELEMETRY:
        motorTelemetry.current_q[1] = data1;
        motorTelemetry.current_d[1] = data2;
        recordCANTelemetry(1); // Record successful time of communication from Motor 2
        Serial.println("Motor 2 current telemetry packet has been received by ESP32");
        return true;

    // Motor 2 dynamic package
    case CAN_ID_MOTOR2_DYNAMIC_TELEMETRY:
        motorTelemetry.velocity[1] = data1;
        motorTelemetry.angle[1] = data2;
        recordCANTelemetry(1); // Record successful time of communication from Motor 2
        Serial.println("Motor 2 dynamic telemetry packet has been received by ESP32");
        return true;

    // Motor 3 current package
    case CAN_ID_MOTOR3_CURRENT_TELEMETRY:
        motorTelemetry.current_q[2] = data1;
        motorTelemetry.current_d[2] = data2;
        recordCANTelemetry(2); // Record successful time of communication from Motor 3 
        Serial.println("Motor 3 current telemetry packet has been received by ESP32");
        return true;

    // Motor 3 dynamic package
    case CAN_ID_MOTOR3_DYNAMIC_TELEMETRY:
        motorTelemetry.velocity[2] = data1;
        motorTelemetry.angle[2] = data2;
        recordCANTelemetry(2); // Record successful time of communication from Motor 3
        Serial.println("Motor 3 dynamic telemetry packet has been received by ESP32");
        return true;

    // Motor 4 current package    
    case CAN_ID_MOTOR4_CURRENT_TELEMETRY:
        motorTelemetry.current_q[3] = data1;
        motorTelemetry.current_d[3] = data2;
        recordCANTelemetry(3); // Record successful time of communication from Motor 4
        Serial.println("Motor 4 current telemetry packet has been received by ESP32");
        return true;

    // Motor 4 dynamic package
    case CAN_ID_MOTOR4_DYNAMIC_TELEMETRY:
        motorTelemetry.velocity[3] = data1;
        motorTelemetry.angle[3] = data2;
        recordCANTelemetry(3); // Record successful time of communication from Motor 4
        Serial.println("Motor 4 dynamic telemetry packet has been received by ESP32"); 
        return true;

    // If package ID is unrecognized
    default:
        Serial.println("Unknown motor telemetry CAN id #RIP");
        break;

  }


}

*/
// Check CAN bus status and communication errors.
//bool checkCANStatus()	{

//

