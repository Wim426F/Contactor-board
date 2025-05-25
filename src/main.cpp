#include <Arduino.h>
#include <FlexCAN_T4.h>

// Define pins
#define CCSN_IN 25
#define CCSP_IN 24
#define MCONN_IN 20
#define MCONP_IN 21
#define PRECHARGE_IN 16
#define REGEN_IN 17

#define MCONP_GATE 19
#define MCONN_GATE 18
#define CCSP_GATE 13
#define CCSN_GATE 14
#define PRECHARGE 12

// Define states for contactors
enum ContactorState { OFF, STARTUP, ECONOMIZED };
ContactorState ccsnState = OFF, ccspState = OFF, mconpState = OFF, mconnState = OFF;

// Define system states for CAN bus mode
enum SystemState { IDLE, PRECHARGING, PRECHARGE_WAIT, CONTACTORS_ON };
SystemState systemState = IDLE;

// Define precharge status for debug message
enum PrechargeStatus { PRECHARGE_IDLE, PRECHARGE_IN_PROGRESS, PRECHARGE_SUCCESS, PRECHARGE_FAILED_TIMEOUT };
PrechargeStatus prechargeStatus = PRECHARGE_IDLE;

// PWM settings
const uint16_t PWM_FREQ = 20000; // 20 kHz
const uint8_t ECONOMY_DUTY_CYCLE = 30; // 30%
const uint16_t FULL_CURRENT_TIME = 500; // 500 ms latch-on time

// CAN settings
#define CAN_ID_INPUT 0x108    // VCU HV enable request (CUSTOM-VEHICLE-CAN) (typically from PCS controller)
#define CAN_ID_OUTPUT 0x398   // Status output (CUSTOM-VEHICLE-CAN)
#define CAN_ID_GFM 0x293      // GFM v2 CANOpen ID (CUSTOM-VEHICLE-CAN)
#define CAN_ID_BMS 0x132      // BMS voltage/current (ID132HVBattAmpVolt, TESLA-PRTY-CAN)
#define CAN_ID_MOTOR 0x126    // Motor HV voltage (ID126RearHVStatus, TESLA-VEHICLE-CAN)
#define CAN_ID_MAX_POWER 0x696 // T2C power limit (TESLA-VEHICLE-CAN)
#define CAN_ID_SHIFT 0x697     // T2C shift command (TESLA-VEHICLE-CAN)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;  // TESLA-VEHICLE-CAN: T2C and drive unit
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;  // CUSTOM-VEHICLE-CAN: GFM, VCU, etc.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;  // TESLA-PRTY-CAN: T2C, BMS, drive unit stats
CAN_message_t rxMsg, txMsg;

// Timing variables
unsigned long lastCanSendTime = 0;
const unsigned long CAN_SEND_INTERVAL = 100; // 100 ms
unsigned long ccsnStartTime = 0, ccspStartTime = 0;
unsigned long mconpStartTime = 0, mconnStartTime = 0;
unsigned long prechargeStartTime = 0;
const unsigned long PRECHARGE_TIMEOUT = 5000; // 5 seconds

// BMS and Motor variables
float batteryVoltage = 0.0;      // From BMS (BattVoltage132, Can3)
float rearMotorVoltage = 0.0;    // From Motor (RearHighVoltage126, Can1)
uint8_t rearMotorVoltageQF = 0;  // Quality flag for motor voltage
const float PRECHARGE_VOLTAGE_THRESHOLD = 0.95; // 95% match

// GFM variables
uint16_t gfmIsolation = 0; // ohms/volt, default to fault
const uint16_t GFM_ISO_THRESHOLD = 500; // Minimum safe isolation (ohms/volt)

// Function prototypes
void handleInput(uint8_t inputPin, uint8_t outputPin, ContactorState &state, unsigned long &startTime);
void handleMconGate(ContactorState &state, bool enable, uint8_t outputPin, unsigned long &startTime);
void economizeGate(uint8_t outputPin, ContactorState &state);
void sendStateViaCAN();
bool checkGroundFault();
void processOverrideMode();
void processCanBusMode();
void gatewayMessages();
void processBmsAndMotorMessages();

void setup() {
  // Initialize pins
  pinMode(CCSN_IN, INPUT_PULLDOWN);
  pinMode(CCSP_IN, INPUT_PULLDOWN);
  pinMode(MCONP_IN, INPUT_PULLDOWN);
  pinMode(MCONN_IN, INPUT_PULLDOWN);
  pinMode(PRECHARGE_IN, INPUT_PULLDOWN);
  pinMode(REGEN_IN, INPUT_PULLDOWN);
  
  pinMode(MCONP_GATE, OUTPUT);
  pinMode(MCONN_GATE, OUTPUT);
  pinMode(CCSP_GATE, OUTPUT);
  pinMode(CCSN_GATE, OUTPUT);
  pinMode(PRECHARGE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(MCONP_GATE, LOW);
  digitalWrite(MCONN_GATE, LOW);
  digitalWrite(CCSP_GATE, LOW);
  digitalWrite(CCSN_GATE, LOW);
  digitalWrite(PRECHARGE, LOW);

  // Set PWM frequency and resolution for contactor pins (excluding PRECHARGE)
  analogWriteFrequency(MCONP_GATE, PWM_FREQ);
  analogWriteFrequency(MCONN_GATE, PWM_FREQ);
  analogWriteFrequency(CCSP_GATE, PWM_FREQ);
  analogWriteFrequency(CCSN_GATE, PWM_FREQ);
  analogWriteResolution(8); // 8-bit resolution for 0-255 range

  // Initialize CAN buses
  Can1.begin();
  Can1.setBaudRate(500000); // TESLA-VEHICLE-CAN (T2C/drive unit)
  Can1.setMBFilter(MB0, CAN_ID_MAX_POWER); // Filter for power limit
  Can1.setMBFilter(MB1, CAN_ID_SHIFT);     // Filter for shift command
  Can1.setMBFilter(MB2, CAN_ID_MOTOR);     // Filter for motor messages

  Can2.begin();
  Can2.setBaudRate(500000); // CUSTOM-VEHICLE-CAN (GFM, VCU, etc.)
  Can2.setMBFilter(MB0, CAN_ID_GFM);       // Filter for GFM messages
  Can2.setMBFilter(MB1, CAN_ID_INPUT);     // Filter for VCU HV enable
  Can2.setMBFilter(MB2, CAN_ID_MAX_POWER); // Filter for T2C power limit to gateway
  Can2.setMBFilter(MB3, CAN_ID_SHIFT);     // Filter for T2C shift to gateway

  Can3.begin();
  Can3.setBaudRate(500000); // TESLA-PRTY-CAN (BMS)
  Can3.setMBFilter(MB0, CAN_ID_BMS);       // Filter for BMS messages

  // Initialize CAN output message
  txMsg.id = CAN_ID_OUTPUT;
  txMsg.len = 6; // 6 bytes: 4 contactor states + regen status + precharge status
  txMsg.flags.extended = 0; // Standard ID
}

void loop() {
  // Process BMS and Motor messages (update voltages and relay to Can2)
  processBmsAndMotorMessages();

  // Gateway other messages from Can2 to Can1
  gatewayMessages();

  // Check for override mode (hardware inputs)
  if (digitalRead(PRECHARGE_IN) || digitalRead(MCONP_IN) || digitalRead(MCONN_IN)) {
    processOverrideMode();
  } else {
    processCanBusMode();
  }

  // Handle CCSN and CCSP (always via hardware inputs)
  handleInput(CCSN_IN, CCSN_GATE, ccsnState, ccsnStartTime);
  handleInput(CCSP_IN, CCSP_GATE, ccspState, ccspStartTime);

  // Send status via CAN every 100 ms
  if (millis() - lastCanSendTime >= CAN_SEND_INTERVAL) {
    lastCanSendTime = millis();
    sendStateViaCAN();
    digitalToggle(LED_BUILTIN);
  }
}

void processBmsAndMotorMessages() {
  // Drive unit message (ID126RearHVStatus) from Can1
  if (Can1.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_MOTOR) {
      rearMotorVoltage = (rxMsg.buf[0] | (rxMsg.buf[1] & 0x03) << 8) * 0.5; // Bits 0-9, 0.5 V/bit
      rearMotorVoltageQF = (rxMsg.buf[1] >> 2) & 0x01; // Bit 10
      Can2.write(rxMsg); // Relay to Can2 for VCU
    }
  }

  // BMS message (ID132HVBattAmpVolt) from Can3
  if (Can3.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_BMS) {
      batteryVoltage = ((rxMsg.buf[0] << 8) | rxMsg.buf[1]) * 0.01; // Bits 0-15, 0.01 V/bit
      Can2.write(rxMsg); // Relay to Can2 for VCU
    }
  }

  // GFM message from Can2
  if (Can2.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_GFM) {
      gfmIsolation = (rxMsg.buf[2] << 8) | rxMsg.buf[3]; // Bits 16-31, ohms/volt
    }
  }
}

void processOverrideMode() {
  // Check ground fault
  if (!checkGroundFault()) {
    digitalWrite(PRECHARGE, LOW);
    handleMconGate(mconpState, false, MCONP_GATE, mconpStartTime);
    handleMconGate(mconnState, false, MCONN_GATE, mconnStartTime);
    systemState = IDLE;
    prechargeStatus = PRECHARGE_IDLE;
    return;
  }

  // Respect hardware inputs directly
  digitalWrite(PRECHARGE, digitalRead(PRECHARGE_IN));
  handleMconGate(mconpState, digitalRead(MCONP_IN), MCONP_GATE, mconpStartTime);
  handleMconGate(mconnState, digitalRead(MCONN_IN), MCONN_GATE, mconnStartTime);
}

void processCanBusMode() {
  static bool hvRequested = false;

  // Check for CAN input from VCU on Can2
  if (Can2.read(rxMsg) && rxMsg.id == CAN_ID_INPUT) {
    // 0xAA == HV request ON, 0xCC == HV request OFF
    hvRequested = (rxMsg.buf[0] == 0xAA);
  }

  // State machine for CAN bus mode
  switch (systemState) {
    case IDLE:
      if (hvRequested && checkGroundFault()) {
        systemState = PRECHARGING;
      } else {
        digitalWrite(PRECHARGE, LOW);
        handleMconGate(mconpState, false, MCONP_GATE, mconpStartTime);
        handleMconGate(mconnState, false, MCONN_GATE, mconnStartTime);
        prechargeStatus = PRECHARGE_IDLE;
      }
      break;

    case PRECHARGING:
      digitalWrite(PRECHARGE, HIGH);
      prechargeStartTime = millis();
      prechargeStatus = PRECHARGE_IN_PROGRESS;
      systemState = PRECHARGE_WAIT;
      break;

    case PRECHARGE_WAIT:
      if (!checkGroundFault()) {
        digitalWrite(PRECHARGE, LOW);
        prechargeStatus = PRECHARGE_IDLE;
        systemState = IDLE;
      } else if (rearMotorVoltageQF == 1 && batteryVoltage > 0 && rearMotorVoltage >= batteryVoltage * PRECHARGE_VOLTAGE_THRESHOLD) {
        digitalWrite(PRECHARGE, LOW);
        prechargeStatus = PRECHARGE_SUCCESS;
        systemState = CONTACTORS_ON;
      } else if (millis() - prechargeStartTime >= PRECHARGE_TIMEOUT) {
        digitalWrite(PRECHARGE, LOW);
        prechargeStatus = PRECHARGE_FAILED_TIMEOUT;
        systemState = IDLE;
      }
      break;

    case CONTACTORS_ON:
      if (!checkGroundFault()) {
        handleMconGate(mconpState, false, MCONP_GATE, mconpStartTime);
        handleMconGate(mconnState, false, MCONN_GATE, mconnStartTime);
        prechargeStatus = PRECHARGE_IDLE;
        systemState = IDLE;
      } else {
        handleMconGate(mconpState, rxMsg.buf[0] == 1, MCONP_GATE, mconpStartTime);
        handleMconGate(mconnState, rxMsg.buf[1] == 1, MCONN_GATE, mconnStartTime);
      }
      break;
  }
}

bool checkGroundFault() {
  return gfmIsolation >= GFM_ISO_THRESHOLD; // True if no fault (above 500 ohms/volt)
}

void handleInput(uint8_t inputPin, uint8_t outputPin, ContactorState &state, unsigned long &startTime) {
  if (digitalRead(inputPin)) {
    if (state == OFF) {
      state = STARTUP;
      analogWrite(outputPin, 255); // 100% ON
      startTime = millis();
    }
    if (state == STARTUP && millis() - startTime >= FULL_CURRENT_TIME) {
      economizeGate(outputPin, state);
    }
  } else {
    if (state != OFF) {
      analogWrite(outputPin, 0); // OFF
      state = OFF;
    }
  }
}

void handleMconGate(ContactorState &state, bool enable, uint8_t outputPin, unsigned long &startTime) {
  if (enable) {
    if (state == OFF) {
      state = STARTUP;
      analogWrite(outputPin, 255); // 100% ON
      startTime = millis();
    }
    if (state == STARTUP && millis() - startTime >= FULL_CURRENT_TIME) {
      economizeGate(outputPin, state);
    }
  } else {
    if (state != OFF) {
      analogWrite(outputPin, 0); // OFF
      state = OFF;
    }
  }
}

void economizeGate(uint8_t outputPin, ContactorState &state) {
  state = ECONOMIZED;
  analogWrite(outputPin, ECONOMY_DUTY_CYCLE * 255 / 100);
}

void sendStateViaCAN() {  
  // send to VCU 
  txMsg.buf[0] = mconpState;
  txMsg.buf[1] = mconnState;
  txMsg.buf[2] = ccspState;
  txMsg.buf[3] = ccsnState;
  txMsg.buf[4] = digitalRead(REGEN_IN); // Regen status from T2C
  txMsg.buf[5] = prechargeStatus;
  Can2.write(txMsg); // Send on CUSTOM-VEHICLE-CAN
}

void gatewayMessages() {
  // Tunnel messages from VCU on Can2 to T2C on Can1
  if (Can2.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_MAX_POWER || rxMsg.id == CAN_ID_SHIFT) {
      Can1.write(rxMsg); // Forward to TESLA-VEHICLE-CAN
    }
  }
}