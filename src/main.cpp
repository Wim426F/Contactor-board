#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Snooze.h>

// CAN settings
#define CAN_ID_HVREQ          0x397 // VCU HV request (POWERTRAIN-CAN)
#define CAN_ID_STATUS         0x398 // Status output (POWERTRAIN-CAN)
#define CAN_ID_MOTOR          0x126 // Motor HV voltage (ID126RearHVStatus, TESLA-VEHICLE-CAN)
#define CAN_ID_MAX_POWER      0x696 // T2C power limit (TESLA-VEHICLE-CAN)
#define CAN_ID_SHIFT          0x697 // T2C shift command (TESLA-VEHICLE-CAN)
#define CAN_ID_INVERTER_TEMP  0x315 // ID315RearInverterTemps
#define CAN_ID_DRIVE_STAT     0x118 // ID118DriveSystemStatus
#define CAN_ID_REAR_POWER     0x266 // ID266RearInverterPower
#define CAN_ID_SYSTEM_POWER   0x268 // ID268SystemPower
#define CAN_ID_MOTOR_TORQUE   0x108 // ID264DIR_torque
#define CAN_ID_MOTOR_TORQUE_NEW 0x107 // ID264DIR_torque renamed

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;  // POWERTRAIN-CAN:     GFM, VCU, BMS etc.  CAN2 in hardware
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;  // TESLA-PRTY-CAN:     T2C, drive unit     CAN3 in hardware
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can4;  // TESLA-VEHICLE-CAN:  T2C, drive unit     CAN4 in hardware (why tf did i do this:/
CAN_message_t rxMsg, txMsg;

// Inputs
#define CCSN_IN     25
#define CCSP_IN     24
#define KEYON_IN    2

// Outputs
#define MCONP_GATE  19
#define MCONN_GATE  18
#define CCSP_GATE   13
#define CCSN_GATE   14
#define PRECHARGE   12

enum ContactorState { OFF, STARTUP, ECONOMIZED };

ContactorState ccsnState = OFF, ccspState = OFF, mconpState = OFF, mconnState = OFF;

// PWM setting
const uint16_t PWM_FREQ = 20000;                // 20 kHz
const uint8_t ECONOMY_DUTY_CYCLE = 40;          // 40%
const uint16_t FULL_CURRENT_TIME = 500;         // 500 ms latch-on time

// Timing variables
const unsigned long CAN_SEND_INTERVAL = 100;
unsigned long lastCanSendTime = 0;
unsigned long ccsnStartTime = 0, ccspStartTime = 0;
unsigned long mconpStartTime = 0, mconnStartTime = 0;

// HVCU control states from CAN
bool prechargeEnable = false;
bool mconnEnable = false;
bool mconpEnable = false;

SnoozeDigital digital; // For pin wake
SnoozeBlock config(digital); // Install driver

void handleCANMessages();
void handleCCScontactor(uint8_t inputPin, uint8_t outputPin, ContactorState &state, unsigned long &startTime);
void handleMainContactor(ContactorState &state, bool enable, uint8_t outputPin, unsigned long &startTime);
void economizeGate(uint8_t outputPin, ContactorState &state);
void sendStateViaCAN();
void enterLowPower();

void initCAN() {
  // POWERTRAIN-CAN (GFM, VCU, etc.)
  Can2.begin();
  Can2.setBaudRate(500000); 
  Can2.setMBFilter(MB0, CAN_ID_HVREQ);     // Filter for HVCU control messages
  Can2.setMBFilter(MB1, CAN_ID_MAX_POWER); // Filter for T2C power limit to gateway
  Can2.setMBFilter(MB2, CAN_ID_SHIFT);     // Filter for T2C shift to gateway

  // TESLA-PRTY-CAN (BMS)
  Can3.begin();
  Can3.setBaudRate(500000); 

  // TESLA-VEHICLE-CAN (T2C/drive unit)
  Can4.begin();
  Can4.setBaudRate(500000); 
  Can4.setMBFilter(MB0, CAN_ID_MOTOR);     // Filter for motor messages
  Can4.setMBFilter(MB1, CAN_ID_MOTOR_TORQUE);     // Filter for motor messages
  Can4.setMBFilter(MB2, CAN_ID_INVERTER_TEMP);     // Filter for motor messages
  Can4.setMBFilter(MB3, CAN_ID_DRIVE_STAT);     // Filter for motor messages
  Can4.setMBFilter(MB4, CAN_ID_REAR_POWER);     // Filter for motor messages
  Can4.setMBFilter(MB5, CAN_ID_SYSTEM_POWER);     // Filter for motor messages
}

void initPWM() {
  // Set PWM frequency and resolution for contactor pins (excluding PRECHARGE)
  analogWriteFrequency(MCONP_GATE, PWM_FREQ);
  analogWriteFrequency(MCONN_GATE, PWM_FREQ);
  analogWriteFrequency(CCSP_GATE, PWM_FREQ);
  analogWriteFrequency(CCSN_GATE, PWM_FREQ);
  analogWriteResolution(8); // 8-bit resolution for 0-255 range
}

void setup() {
  // Initialize pins
  pinMode(CCSN_IN, INPUT_PULLDOWN);
  pinMode(CCSP_IN, INPUT_PULLDOWN);
  pinMode(KEYON_IN, INPUT_PULLDOWN);

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

  initPWM(); // Init PWM settings

  initCAN(); // Init all CAN buses and filters

  txMsg.id = CAN_ID_STATUS;
  txMsg.len = 7;
  txMsg.flags.extended = 0;

  // Snooze config for wake on KEYON_IN rising
  digital.pinMode(KEYON_IN, INPUT_PULLDOWN, RISING);
}

void loop() {
  handleCANMessages();

  // Handle CCS contactors based on hardware inputs
  handleCCScontactor(CCSN_IN, CCSN_GATE, ccsnState, ccsnStartTime);
  handleCCScontactor(CCSP_IN, CCSP_GATE, ccspState, ccspStartTime);

  // Handle main contactors and precharge based on latest CAN command
  digitalWrite(PRECHARGE, prechargeEnable ? HIGH : LOW);
  handleMainContactor(mconnState, mconnEnable, MCONN_GATE, mconnStartTime);
  handleMainContactor(mconpState, mconpEnable, MCONP_GATE, mconpStartTime);

  if (millis() - lastCanSendTime >= CAN_SEND_INTERVAL) {
    lastCanSendTime = millis();
    sendStateViaCAN();
    digitalToggle(LED_BUILTIN);
  }

  // Sleep if KEYON low and contactors OFF
  if (digitalRead(KEYON_IN) == LOW && mconnState == OFF && mconpState == OFF) {
    enterLowPower();
  }
}

void handleCANMessages() {
  // CUSTOM POWERTRAIN-CAN:
  if (Can2.read(rxMsg)) {
    // Tunnel messages from VCU on Can2 to T2C on Can4
    if (rxMsg.id == CAN_ID_MAX_POWER || rxMsg.id == CAN_ID_SHIFT) {
      Can4.write(rxMsg); // Forward to TESLA-VEHICLE-CAN
    }

    // VCU HVCU control message: [0x39, precharge, negative, positive]
    if (rxMsg.id == CAN_ID_HVREQ && rxMsg.len == 4 && rxMsg.buf[0] == 0x39) {
      // Update only for valid ON (0x02) or OFF (0x03) commands
      if (rxMsg.buf[1] == 0x02) prechargeEnable = true;
      else if (rxMsg.buf[1] == 0x03) prechargeEnable = false;
      if (rxMsg.buf[2] == 0x02) mconnEnable = true;
      else if (rxMsg.buf[2] == 0x03) mconnEnable = false;
      if (rxMsg.buf[3] == 0x02) mconpEnable = true;
      else if (rxMsg.buf[3] == 0x03) mconpEnable = false;
    }
  }

  // TESLA-VEHICLE-CAN
  if (Can4.read(rxMsg)) {
    // Tunnel messages from T2C on Can4 to VCU on Can2
    if (rxMsg.id == CAN_ID_MOTOR_TORQUE) {
      rxMsg.id = CAN_ID_MOTOR_TORQUE_NEW; // rename msg ID to avoid collision with PCS controller
      Can2.write(rxMsg); // Forward to POWERTRAIN with the new ID
    }
    else
    {
      Can2.write(rxMsg); // Forward to POWERTRAIN
    }
  }
}

void handleCCScontactor(uint8_t inputPin, uint8_t outputPin, ContactorState &state, unsigned long &startTime) {
  if (digitalRead(inputPin)) {
    if (state == OFF) {
      state = STARTUP;
      analogWrite(outputPin, 255);
      startTime = millis();
    }
    if (state == STARTUP && millis() - startTime >= FULL_CURRENT_TIME) {
      economizeGate(outputPin, state);
    }
  } else {
    if (state != OFF) {
      analogWrite(outputPin, 0);
      state = OFF;
    }
  }
}

void handleMainContactor(ContactorState &state, bool enable, uint8_t outputPin, unsigned long &startTime) {
  if (enable) {
    if (state == OFF) {
      state = STARTUP;
      analogWrite(outputPin, 255);
      startTime = millis();
    }
    if (state == STARTUP && millis() - startTime >= FULL_CURRENT_TIME) {
      economizeGate(outputPin, state);
    }
  } else {
    if (state != OFF) {
      analogWrite(outputPin, 0);
      state = OFF;
    }
  }
}

void economizeGate(uint8_t outputPin, ContactorState &state) {
  state = ECONOMIZED;
  analogWrite(outputPin, ECONOMY_DUTY_CYCLE * 255 / 100);
}

void sendStateViaCAN() {
  txMsg.buf[0] = mconpState;
  txMsg.buf[1] = mconnState;
  txMsg.buf[2] = ccspState;
  txMsg.buf[3] = ccsnState;
  txMsg.len = 4;
  Can2.write(txMsg);
}

void enterLowPower() {
  // Prep: LED off
  digitalWrite(LED_BUILTIN, LOW);

  // Deep sleep, wake on KEYON rising
  Snooze.deepSleep(config);

  // Post-wake: Re-init
  initPWM();
  initCAN();
}