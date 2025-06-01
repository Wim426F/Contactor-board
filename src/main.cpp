#include <Arduino.h>
#include <FlexCAN_T4.h>

// CAN settings
#define CAN_ID_INPUT          0x108 // VCU HV enable request (POWERTRAIN-CAN)
#define CAN_ID_STATUS         0x398 // Status output (POWERTRAIN-CAN)
#define CAN_ID_GFM            0x293 // GFM v2 CANOpen ID (POWERTRAIN-CAN)
#define CAN_ID_BMS            0x293 // BMS voltage/current (ID132HVBattAmpVolt, TESLA-PRTY-CAN)
#define CAN_ID_MOTOR          0x126 // Motor HV voltage (ID126RearHVStatus, TESLA-VEHICLE-CAN)
#define CAN_ID_MAX_POWER      0x696 // T2C power limit (TESLA-VEHICLE-CAN)
#define CAN_ID_SHIFT          0x697 // T2C shift command (TESLA-VEHICLE-CAN)
#define CAN_ID_INVERTER_TEMP  0x315 // ID315RearInverterTemps
#define CAN_ID_MOTOR_TORQUE   0x108 // ID264DIR_torque
#define CAN_ID_MOTOR_TORQUE_NEW   0x801 // ID264DIR_torque renamed

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;  // TESLA-VEHICLE-CAN:  T2C and drive unit
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;  // POWERTRAIN-CAN:     GFM, VCU, (BMS as well) etc.
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;  // TESLA-PRTY-CAN:     T2C, BMS(MCU), drive unit stats
CAN_message_t rxMsg, txMsg;

// Inputs
#define CCSN_IN     25
#define CCSP_IN     24
#define MCONN_IN    20
#define MCONP_IN    21
#define BMS_OK_IN   16
#define REGEN_IN    17
// Outputs
#define MCONP_GATE  19
#define MCONN_GATE  18
#define CCSP_GATE   13
#define CCSN_GATE   14
#define PRECHARGE   12

enum ContactorState { OFF, STARTUP, ECONOMIZED };
enum SystemState { IDLE, PRECHARGING, CONTACTORS_ON, PRECHARGE_FAILED_TIMEOUT, PRECHARGE_BLOCKED_BY_BMS, PRECHARGE_BLOCKED_BY_GFM };

ContactorState ccsnState = OFF, ccspState = OFF, mconpState = OFF, mconnState = OFF;
SystemState systemState = IDLE;

// PWM setting
const uint16_t PWM_FREQ = 20000;                // 20 kHz
const uint8_t ECONOMY_DUTY_CYCLE = 30;          // 30%
const uint16_t FULL_CURRENT_TIME = 500;         // 500 ms latch-on time
const unsigned long PRECHARGE_TIMEOUT = 5000;   // 5 seconds
const float PRECHARGE_VOLTAGE_THRESHOLD = 0.95; // 95% voltage match for precharge OK
const uint16_t GFM_ISO_THRESHOLD = 500;         // 500 Ohms per volt
const float BMS_SAFE_CURRENT = 20.0;            // Only open contactors below this current

// Timing variables
const unsigned long CAN_SEND_INTERVAL = 100;
unsigned long lastCanSendTime = 0;
unsigned long ccsnStartTime = 0, ccspStartTime = 0;
unsigned long mconpStartTime = 0, mconnStartTime = 0;
unsigned long prechargeStartTime = 0;
unsigned long lastBmsReplyTime = 0;
const unsigned long BMS_REPLY_TIMEOUT = 200;
unsigned long shutdownRequestTime = 0;
bool shutdownPending = false;

// BMS and Motor variables
float batteryVoltage = 0.0;     // From BMS (BattVoltage132, Can3)
float rearMotorVoltage = 0.0;   // From Motor (RearHighVoltage126, Can1)
uint8_t rearMotorVoltageQF = 0; // Quality flag for motor voltage
uint16_t gfmIsolation = 0;      // From GFM on Can2
float batteryCurrent = 0.0;     // From BMS on Can3
bool bmsOk = false;
bool isolationOk = false;

bool hvOnRequestedHW = false;
bool hvOnRequestedCAN = false;
bool hvOnRequested = false;


void handleCANMessages();
void requestBmsPackSummary();
void processHVStateMachine();
void handleCCScontactor(uint8_t inputPin, uint8_t outputPin, ContactorState &state, unsigned long &startTime);
void handleMainContactor(ContactorState &state, bool enable, uint8_t outputPin, unsigned long &startTime);
void economizeGate(uint8_t outputPin, ContactorState &state);
void gatewayMessages();
void sendStateViaCAN();

void setup() {
  // Initialize pins
  pinMode(CCSN_IN, INPUT_PULLDOWN);
  pinMode(CCSP_IN, INPUT_PULLDOWN);
  pinMode(MCONP_IN, INPUT_PULLDOWN);
  pinMode(MCONN_IN, INPUT_PULLDOWN);
  pinMode(BMS_OK_IN, INPUT_PULLDOWN);
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
  Can1.setMBFilter(MB3, CAN_ID_MOTOR_TORQUE);     // Filter for motor messages
  Can1.setMBFilter(MB4, CAN_ID_INVERTER_TEMP);     // Filter for motor messages

  Can2.begin();
  Can2.setBaudRate(500000); // POWERTRAIN-CAN (GFM, VCU, etc.)
  Can2.setMBFilter(MB0, CAN_ID_GFM);       // Filter for GFM messages
  Can2.setMBFilter(MB1, CAN_ID_INPUT);     // Filter for VCU HV enable
  Can2.setMBFilter(MB2, CAN_ID_MAX_POWER); // Filter for T2C power limit to gateway
  Can2.setMBFilter(MB3, CAN_ID_SHIFT);     // Filter for T2C shift to gateway

  Can3.begin();
  Can3.setBaudRate(500000); // TESLA-PRTY-CAN (BMS)
  Can3.setMBFilter(MB0, CAN_ID_BMS);       // Filter for BMS messages

  txMsg.id = CAN_ID_STATUS;
  txMsg.len = 7;
  txMsg.flags.extended = 0;
}

void loop() {
  handleCANMessages();
  gatewayMessages();

  bmsOk = digitalRead(BMS_OK_IN);
  isolationOk = (gfmIsolation >= GFM_ISO_THRESHOLD);

  hvOnRequestedHW = digitalRead(MCONP_IN);
  hvOnRequested = hvOnRequestedHW || hvOnRequestedCAN;

  processHVStateMachine();

  handleCCScontactor(CCSN_IN, CCSN_GATE, ccsnState, ccsnStartTime);
  handleCCScontactor(CCSP_IN, CCSP_GATE, ccspState, ccspStartTime);

  if (millis() - lastCanSendTime >= CAN_SEND_INTERVAL) {
    lastCanSendTime = millis();
    sendStateViaCAN();
    digitalToggle(LED_BUILTIN);
    Serial.println("Iso Resistance: " + gfmIsolation);
    Serial.println("");
  }
}

void handleCANMessages() {

  if (Can1.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_MOTOR) {
      rearMotorVoltage = (rxMsg.buf[3] << 8 | rxMsg.buf[2]) * 0.1f;
      rearMotorVoltageQF = rxMsg.buf[1] & 0x01;
    }
  }

  if (Can2.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_INPUT) {
      if (rxMsg.buf[0] == 0xAA) hvOnRequestedCAN = true;
      if (rxMsg.buf[0] == 0xCC) hvOnRequestedCAN = false;
    }
    if (rxMsg.id == CAN_ID_GFM) {
      gfmIsolation = (rxMsg.buf[2] << 8) | rxMsg.buf[3];
    }
  }

  if (Can3.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_BMS && rxMsg.buf[0] == 0x02) {
      batteryVoltage = (rxMsg.buf[3] << 8 | rxMsg.buf[2]) * 0.1f;
      int16_t rawCurrent = (rxMsg.buf[5] << 8) | rxMsg.buf[4];
      batteryCurrent = rawCurrent * 0.1f;
      lastBmsReplyTime = millis();
    }
  }
}

void processHVStateMachine() {
  static bool bmsFaultWhileOn = false;
  static bool hvReqDropWhileOn = false;
  static bool prevHvOnRequested = false;

  switch (systemState) {
    case IDLE:
      bmsFaultWhileOn = false;
      hvReqDropWhileOn = false;
      shutdownPending = false;
      if (hvOnRequested && isolationOk && bmsOk) {
        requestBmsPackSummary(); //get battery voltage as required for precharging.
        systemState = PRECHARGING;
        digitalWrite(PRECHARGE, HIGH);
        prechargeStartTime = millis();
      } else if (hvOnRequested && !isolationOk) {
        systemState = PRECHARGE_BLOCKED_BY_GFM;
        digitalWrite(PRECHARGE, LOW);
        handleMainContactor(mconpState, false, MCONP_GATE, mconpStartTime);
        handleMainContactor(mconnState, false, MCONN_GATE, mconnStartTime);
      } else if (hvOnRequested && !bmsOk) {
        systemState = PRECHARGE_BLOCKED_BY_BMS;
        digitalWrite(PRECHARGE, LOW);
        handleMainContactor(mconpState, false, MCONP_GATE, mconpStartTime);
        handleMainContactor(mconnState, false, MCONN_GATE, mconnStartTime);
      } else {
        digitalWrite(PRECHARGE, LOW);
        handleMainContactor(mconpState, false, MCONP_GATE, mconpStartTime);
        handleMainContactor(mconnState, false, MCONN_GATE, mconnStartTime);
      }
      break;

    case PRECHARGING:
      if (!isolationOk) {
        digitalWrite(PRECHARGE, LOW);
        systemState = PRECHARGE_BLOCKED_BY_GFM;
      } else if (!bmsOk) {
        digitalWrite(PRECHARGE, LOW);
        systemState = PRECHARGE_BLOCKED_BY_BMS;
      } else if (rearMotorVoltageQF == 1 && batteryVoltage > 0 &&
                 rearMotorVoltage >= batteryVoltage * PRECHARGE_VOLTAGE_THRESHOLD) {
        digitalWrite(PRECHARGE, LOW);
        systemState = CONTACTORS_ON;
      } else if (millis() - prechargeStartTime >= PRECHARGE_TIMEOUT) {
        digitalWrite(PRECHARGE, LOW);
        systemState = PRECHARGE_FAILED_TIMEOUT;
      }
      break;

    case CONTACTORS_ON:
      if (!bmsOk)
        bmsFaultWhileOn = true;
      if (!hvOnRequested && prevHvOnRequested)
        hvReqDropWhileOn = true;

      if ((bmsFaultWhileOn || hvReqDropWhileOn)) {
        if (!shutdownPending) {
          requestBmsPackSummary();
          shutdownRequestTime = millis();
          shutdownPending = true;
        }
        if ((millis() - lastBmsReplyTime < BMS_REPLY_TIMEOUT) && fabs(batteryCurrent) < BMS_SAFE_CURRENT) {
          handleMainContactor(mconpState, false, MCONP_GATE, mconpStartTime);
          handleMainContactor(mconnState, false, MCONN_GATE, mconnStartTime);
          systemState = IDLE;
          shutdownPending = false;
        }
      } else {
        shutdownPending = false;
        handleMainContactor(mconpState, true, MCONP_GATE, mconpStartTime);
        handleMainContactor(mconnState, true, MCONN_GATE, mconnStartTime);
      }
      break;

    case PRECHARGE_BLOCKED_BY_GFM:
      handleMainContactor(mconpState, false, MCONP_GATE, mconpStartTime);
      handleMainContactor(mconnState, false, MCONN_GATE, mconnStartTime);
      digitalWrite(PRECHARGE, LOW);
      if (!hvOnRequested)
        systemState = IDLE;
      else if (hvOnRequested && isolationOk && bmsOk) {
        systemState = PRECHARGING;
        digitalWrite(PRECHARGE, HIGH);
        prechargeStartTime = millis();
      }
      break;

    case PRECHARGE_BLOCKED_BY_BMS:
      handleMainContactor(mconpState, false, MCONP_GATE, mconpStartTime);
      handleMainContactor(mconnState, false, MCONN_GATE, mconnStartTime);
      digitalWrite(PRECHARGE, LOW);
      if (!hvOnRequested)
        systemState = IDLE;
      else if (hvOnRequested && isolationOk && bmsOk) {
        systemState = PRECHARGING;
        digitalWrite(PRECHARGE, HIGH);
        prechargeStartTime = millis();
      }
      break;

    case PRECHARGE_FAILED_TIMEOUT:
      handleMainContactor(mconpState, false, MCONP_GATE, mconpStartTime);
      handleMainContactor(mconnState, false, MCONN_GATE, mconnStartTime);
      digitalWrite(PRECHARGE, LOW);
      if (!hvOnRequested)
        systemState = IDLE;
      break;
  }

  prevHvOnRequested = hvOnRequested;
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
  uint8_t hvReqSource = 0;
  if (hvOnRequestedCAN) hvReqSource |= 0x01;
  if (hvOnRequestedHW) hvReqSource |= 0x02;

  txMsg.buf[0] = mconpState;
  txMsg.buf[1] = mconnState;
  txMsg.buf[2] = ccspState;
  txMsg.buf[3] = ccsnState;
  txMsg.buf[4] = digitalRead(REGEN_IN);
  txMsg.buf[5] = systemState;
  txMsg.buf[6] = hvReqSource;
  txMsg.len = 7;
  Can2.write(txMsg);
}

void requestBmsPackSummary() {  //PDO2 MOSI (SID 0x313) – MCU Status Data Request
  CAN_message_t bmsRequest;
  bmsRequest.id = 0x313;
  bmsRequest.len = 8;
  bmsRequest.flags.extended = 0;
  bmsRequest.buf[0] = 0x00;
  bmsRequest.buf[1] = 0x00;
  bmsRequest.buf[2] = 0x00;
  bmsRequest.buf[3] = 0x00;
  bmsRequest.buf[4] = 0x00;
  bmsRequest.buf[5] = 0x00;
  bmsRequest.buf[6] = 0x00;
  bmsRequest.buf[7] = 0x00;
  Can3.write(bmsRequest);
}

void gatewayMessages() {
  // Tunnel messages from T2C on Can1 to VCU on Can2
  if (Can1.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_INVERTER_TEMP || rxMsg.id == CAN_ID_MOTOR) {
      Can2.write(rxMsg); // Forward to POWERTRAIN
    }
    if (rxMsg.id == CAN_ID_MOTOR_TORQUE) {
      rxMsg.id = CAN_ID_MOTOR_TORQUE_NEW; // rename msg ID to avoid collision with PCS controller
      Can2.write(rxMsg); // Forward to POWERTRAIN with the new ID
    }
  }

  // Tunnel messages from VCU on Can2 to T2C on Can1
  if (Can2.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_MAX_POWER || rxMsg.id == CAN_ID_SHIFT) {
      Can1.write(rxMsg); // Forward to TESLA-VEHICLE-CAN
    }
  }
}