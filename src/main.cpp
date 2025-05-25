#include <Arduino.h>
#include <FlexCAN_T4.h>

// CAN settings
#define CAN_ID_INPUT      0x108
#define CAN_ID_STATUS     0x398
#define CAN_ID_GFM        0x293
#define CAN_ID_BMS        0x293
#define CAN_ID_MOTOR      0x126
#define CAN_ID_MAX_POWER  0x696
#define CAN_ID_SHIFT      0x697

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

CAN_message_t rxMsg, txMsg;

//inputs
#define CCSN_IN     25
#define CCSP_IN     24
#define MCONN_IN    20
#define MCONP_IN    21
#define BMS_OK_IN   16
#define REGEN_IN    17
//outputs
#define MCONP_GATE  19
#define MCONN_GATE  18
#define CCSP_GATE   13
#define CCSN_GATE   14
#define PRECHARGE   12

enum ContactorState { OFF, STARTUP, ECONOMIZED };
enum SystemState { IDLE, PRECHARGING, CONTACTORS_ON, PRECHARGE_FAILED_TIMEOUT, PRECHARGE_BLOCKED_BY_BMS, PRECHARGE_BLOCKED_BY_GFM };

ContactorState ccsnState = OFF, ccspState = OFF, mconpState = OFF, mconnState = OFF;
SystemState systemState = IDLE;

const uint16_t PWM_FREQ = 20000;
const uint8_t ECONOMY_DUTY_CYCLE = 30;
const uint16_t FULL_CURRENT_TIME = 500;
const unsigned long PRECHARGE_TIMEOUT = 5000;
const float PRECHARGE_VOLTAGE_THRESHOLD = 0.95;
const uint16_t GFM_ISO_THRESHOLD = 500;
const float BMS_SAFE_CURRENT = 20.0;

unsigned long lastCanSendTime = 0;
const unsigned long CAN_SEND_INTERVAL = 100;
unsigned long ccsnStartTime = 0, ccspStartTime = 0;
unsigned long mconpStartTime = 0, mconnStartTime = 0;
unsigned long prechargeStartTime = 0;

float batteryVoltage = 0.0;
float rearMotorVoltage = 0.0;
uint8_t rearMotorVoltageQF = 0;
uint16_t gfmIsolation = 0;
float batteryCurrent = 0.0;
bool bmsOk = false;
bool isolationOk = false;

bool hvOnRequestedHW = false;
bool hvOnRequestedCAN = false;
bool hvOnRequested = false;

unsigned long lastBmsReplyTime = 0;
const unsigned long BMS_REPLY_TIMEOUT = 200;

bool shutdownPending = false;
unsigned long shutdownRequestTime = 0;

void handleCANMessages();
void requestBmsPackSummary();
void processHVStateMachine();
void handleCCScontactor(uint8_t inputPin, uint8_t outputPin, ContactorState &state, unsigned long &startTime);
void handleMainContactor(ContactorState &state, bool enable, uint8_t outputPin, unsigned long &startTime);
void economizeGate(uint8_t outputPin, ContactorState &state);
void sendStateViaCAN();

void setup() {
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

  analogWriteFrequency(MCONP_GATE, PWM_FREQ);
  analogWriteFrequency(MCONN_GATE, PWM_FREQ);
  analogWriteFrequency(CCSP_GATE, PWM_FREQ);
  analogWriteFrequency(CCSN_GATE, PWM_FREQ);
  analogWriteResolution(8);

  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.setMBFilter(MB0, CAN_ID_MAX_POWER);
  Can1.setMBFilter(MB1, CAN_ID_SHIFT);
  Can1.setMBFilter(MB2, CAN_ID_MOTOR);

  Can2.begin();
  Can2.setBaudRate(500000);
  Can2.setMBFilter(MB0, CAN_ID_GFM);
  Can2.setMBFilter(MB1, CAN_ID_INPUT);

  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.setMBFilter(MB0, CAN_ID_BMS);

  txMsg.id = CAN_ID_STATUS;
  txMsg.len = 7;
  txMsg.flags.extended = 0;
}

void loop() {
  handleCANMessages();

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
  }
}

void handleCANMessages() {
  while (Can2.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_INPUT) {
      if (rxMsg.buf[0] == 0xAA) hvOnRequestedCAN = true;
      if (rxMsg.buf[0] == 0xCC) hvOnRequestedCAN = false;
    }
    if (rxMsg.id == CAN_ID_GFM) {
      gfmIsolation = (rxMsg.buf[2] << 8) | rxMsg.buf[3];
    }
  }
  while (Can1.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_MOTOR) {
      rearMotorVoltage = (rxMsg.buf[3] << 8 | rxMsg.buf[2]) * 0.1f;
      rearMotorVoltageQF = rxMsg.buf[1] & 0x01;
    }
  }
  while (Can3.read(rxMsg)) {
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