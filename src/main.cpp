#include <Arduino.h>
#include <STM32_CAN.h>

// Define pins
#define CCSN_IN PA8
#define CCSP_IN PA9
#define MCONP_GATE PB1
#define MCONN_GATE PB0
#define CCSP_GATE PA6
#define CCSN_GATE PA7

// Define states
enum State { OFF, STARTUP, ECONOMIZED };
State ccsnState = OFF, ccspState = OFF, mconpState = OFF, mconnState = OFF;

// PWM settings
const uint16_t PWM_freq = 20000; // 20 kHz
const uint8_t ECONOMY_DUTY_CYCLE = 30; // 30%
const uint16_t FULL_CURRENT_TIME = 500; // 500 ms

// CAN settings
#define CAN_ID_INPUT 0x397
#define CAN_ID_OUTPUT 0x398
STM32_CAN Can( CAN1, DEF);  //Use PA11/12 pins for CAN1.
static CAN_message_t rxMsg, txMsg;

// Timing variables
unsigned long lastCanSendTime = 0;
const unsigned long CAN_SEND_INTERVAL = 100; // 100 ms
unsigned long lastMconpTime = 0;
unsigned long lastMconnTime = 0;
const unsigned long MCON_TIMEOUT = 1000; // 1000 ms timeout
// Timing variables for non-blocking delays
unsigned long ccsnStartTime = 0, ccspStartTime = 0;
unsigned long mconpStartTime = 0, mconnStartTime = 0;

// Function prototypes
void handleInput(uint8_t inputPin, uint8_t outputPin, State &state, unsigned long &startTime);
void handleMconGate(State &state, bool enable, uint8_t outputPin, unsigned long &startTime);
void economizeGate(uint8_t outputPin, State &state);
void sendStateViaCAN();

void setup() {
  // Initialize pins
  pinMode(CCSN_IN, INPUT_PULLDOWN);
  pinMode(CCSP_IN, INPUT_PULLDOWN);
  pinMode(MCONP_GATE, OUTPUT);
  pinMode(MCONN_GATE, OUTPUT);
  pinMode(CCSP_GATE, OUTPUT);
  pinMode(CCSN_GATE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Can.begin();
  Can.setBaudRate(500000);  //500KBPS
  // Set PWM frequency for STM32
  analogWriteFrequency(PWM_freq);

  // Initialize CAN output message
  txMsg.id = CAN_ID_OUTPUT;
  txMsg.len = 4; // 4 bytes for states
  txMsg.flags.extended = 0; // Standard ID
}

void loop() {
  // Handle inputs for CCSP and CCSN
  handleInput(CCSN_IN, CCSN_GATE, ccsnState, ccsnStartTime);
  handleInput(CCSP_IN, CCSP_GATE, ccspState, ccspStartTime);

  // Check CAN messages for MCONP and MCONN
  if (Can.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_INPUT) {
      handleMconGate(mconpState, rxMsg.buf[0] == 1, MCONP_GATE, mconpStartTime);
      handleMconGate(mconnState, rxMsg.buf[1] == 1, MCONN_GATE, mconnStartTime);
    }
  }

  // Send contactor states via CAN every 100 ms
  if (millis() - lastCanSendTime >= CAN_SEND_INTERVAL) {
    lastCanSendTime = millis();
    sendStateViaCAN();
    digitalToggle(LED_BUILTIN);
  }
}

void handleInput(uint8_t inputPin, uint8_t outputPin, State &state, unsigned long &startTime) {
  if (digitalRead(inputPin)) {
    if (state == OFF) {
      state = STARTUP;
      analogWrite(outputPin, 255); // Set pin to 100% ON
      startTime = millis(); // Start timer
    }
    if (state == STARTUP && millis() - startTime >= FULL_CURRENT_TIME) {
      economizeGate(outputPin, state); // Switch to economized mode
    }
  } else {
    if (state != OFF) {
      analogWrite(outputPin, 0); // Turn pin OFF
      state = OFF;
    }
  }
}

void handleMconGate(State &state, bool enable, uint8_t outputPin, unsigned long &startTime) {
  if (enable) {
    if (state == OFF) {
      state = STARTUP;
      analogWrite(outputPin, 255); // Set pin to 100% ON
      startTime = millis(); // Start timer
    }
    if (state == STARTUP && millis() - startTime >= FULL_CURRENT_TIME) {
      economizeGate(outputPin, state); // Switch to economized mode
    }
  } else {
    if (state != OFF) {
      analogWrite(outputPin, 0); // Turn pin OFF
      state = OFF;
    }
  }
}

// Switches output pin to economized mode
void economizeGate(uint8_t outputPin, State &state) {
  state = ECONOMIZED;
  analogWrite(outputPin, ECONOMY_DUTY_CYCLE * 255 / 100);
}

// Sends the states of the contactors via CAN
void sendStateViaCAN() {
  txMsg.buf[0] = mconpState; // MCONP state
  txMsg.buf[1] = mconnState; // MCONN state
  txMsg.buf[2] = ccspState; // CCSP state
  txMsg.buf[3] = ccsnState; // CCSN state
  
  Can.write(txMsg);
}
