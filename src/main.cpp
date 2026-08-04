#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Snooze.h>

// =====================  CAN IDs  =====================
#define CAN_ID_HVREQ          0x397 // VCU HV request            (CUSTOM POWERTRAIN-CAN, Can2)
#define CAN_ID_STATUS         0x398 // Gateway status output     (CUSTOM POWERTRAIN-CAN, Can2)
#define CAN_ID_TORQUE_CUT     0x201 // VCU torque-cut request    (CUSTOM POWERTRAIN-CAN, Can2)  [NEW]
#define CAN_ID_MAX_POWER      0x696 // T2C power limit input     (relayed Can2 -> T2C)
#define CAN_ID_SHIFT          0x697 // T2C shift command input   (relayed Can2 -> T2C)

// Drive-unit outputs copied to the VCU for diagnostics (Can4 -> Can2):
#define CAN_ID_MOTOR          0x126 // ID126RearHVStatus
#define CAN_ID_INVERTER_TEMP  0x315 // ID315RearInverterTemps
#define CAN_ID_DRIVE_STAT     0x118 // ID118DriveSystemStatus
#define CAN_ID_REAR_POWER     0x266 // ID266RearInverterPower
#define CAN_ID_SYSTEM_POWER   0x268 // ID268SystemPower
#define CAN_ID_DI_ALERTMATRIX 0x3A5 // ID35A_DI_alertMatrix3
#define CAN_ID_DI_LIMITS      0x1D6 // ID1D6DI_limits
#define CAN_ID_REAR_TORQUE    0x1D8 // ID1D8RearTorque
#define CAN_ID_MOTOR_TORQUE     0x108 // ID264DIR_torque
#define CAN_ID_MOTOR_TORQUE_NEW 0x107 // renamed to avoid PCS collision on Can2

// Frame from T2C -> drive unit, that are modified on the fly
#define CAN_ID_POWERTRAIN_CTRL  0x334 // ID334UI_powertrainControl 
#define CAN_ID_CRUISE           0x286 // ID286_DI_locStatus        

// =====================  Torque-cut (0x201) protocol  =====================
//   sender: bytes[0]=cut?MAGIC:0x00 ; bytes[1]=counter&CYCLE ; bytes[2]=crc8(bytes,2,POLY)
#define CAN_CRC_POLY        0xD5   // Comma pedal polynomial
#define REGEN_CUT_FLAG      0x01   // REGEN_CUT_FLAG (bit in byte0)
#define ALL_TORQUE_CUT_FLAG 0x02   // ALL_TORQUE_CUT_FLAG (bit in byte0)
#define COUNTER_CYCLE       0x0F   // rolling counter mask (low nibble)
#define TORQUE_CUT_TIMEOUT_MS 150  // 0x201 is sent every 50 ms; tolerate ~2 misses

// Confirmed on-car: motorOnMode=FRONT_ONLY in 0x334 is IGNORED by the rear DU,
// and 0x334 regenTorqueMax has no effect either (0x334 is advisory). The
// effective power lever is 0x268 (below). Kept at 0; flip only to re-test.
#define ENABLE_EXPERIMENTAL_MOTOR_IDLE 0

// =====================  Bus wiring  =====================
//   Can2 (HW CAN2): CUSTOM POWERTRAIN-CAN - VCU, GFM, BMS, PCS, dashboard
//   Can3 (HW CAN3): TESLA control bus, T2C SEGMENT
//   Can4 (HW CAN1): TESLA control bus, DRIVE-UNIT SEGMENT
// Can3<->Can4 is a fully transparent, accept-all, FIFO-ordered bridge. The only
// frame ever inspected/modified is 0x334, and only its regen byte, only on cut.
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_64> Can2;  // custom bus (mailbox-filtered RX)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_64> Can3;  // T2C segment  (accept-all FIFO RX)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> Can4;  // drive-unit segment (accept-all FIFO RX)
CAN_message_t rxMsg, txMsg;

// Inputs
#define CCSN_IN     25
#define CCSP_IN     24
#define KEYON_IN    2
#define CHGPORT_TEMP A1

// Outputs
#define MCONP_GATE  19
#define MCONN_GATE  18
#define CCSP_GATE   13
#define CCSN_GATE   14
#define PRECHARGE   12
#define CRUISE_EN   16 // legacy regen-disable pin, not used

enum ContactorState { OFF, STARTUP, ECONOMIZED };

ContactorState ccsnState = OFF, ccspState = OFF, mconpState = OFF, mconnState = OFF;

// PWM setting
const uint16_t PWM_FREQ = 20000;                // 20 kHz
const uint8_t ECONOMY_DUTY_CYCLE = 40;          // 40%
const uint16_t FULL_CURRENT_TIME = 500;         // 500 ms latch-on time

// Timing variables
const unsigned long CAN_SEND_INTERVAL = 100;
const unsigned long SLEEP_COUNTDOWN_TIME = 10000; // 10 seconds before sleep
unsigned long lastCanSendTime = 0;
unsigned long ccsnStartTime = 0, ccspStartTime = 0;
unsigned long mconpStartTime = 0, mconnStartTime = 0;
unsigned long sleepCountdownStart = 0;
bool sleepCountdownActive = false;

// HVCU control states from CAN
bool prechargeEnable = false;
bool mconnEnable = false;
bool mconpEnable = false;

// Torque-cut state, driven ONLY by valid 0x201 frames from the VCU.
volatile bool     g_regen_cut     = false; // last valid 0x201: cut regen (braking)
volatile bool     g_all_cut       = false; // last valid 0x201: cut all torque (freeroll)
volatile uint32_t g_last_201_ms   = 0;     // timestamp of last VALID 0x201

SnoozeDigital digital; // For pin wake
SnoozeBlock config(digital); // Install driver

const uint16_t tempSensorTable[71] = {
  4005,3989,3970,3949,3925,3897,3866,3831,3791,3747,
  3698,3643,3583,3518,3446,3369,3286,3198,3104,3005,
  2902,2794,2683,2570,2455,2338,2222,2105,1990,1877,
  1766,1658,1554,1454,1358,1267,1180,1098,1020,947,
  879,815,755,700,648,600,556,515,477,441,
  409,379,351,325,302,280,260,241,224,208,
  194,180,168,157,146,136,127,119,111,104,97
};

void handleCANMessages();
void handleCCScontactor(uint8_t inputPin, uint8_t outputPin, ContactorState &state, unsigned long &startTime);
void handleMainContactor(ContactorState &state, bool enable, uint8_t outputPin, unsigned long &startTime);
void economizeGate(uint8_t outputPin, ContactorState &state);
void sendStateViaCAN();
void enterLowPower();
bool shouldEnterSleep();
void cancelSleepCountdown();

// ---- CRC8 for 0x201. Identical to the VCU sender: poly 0xD5 (Comma pedal),
//      init 0xFF, MSB-first, no reflection, no final xor. ----
static uint8_t crc8(const uint8_t *dat, uint8_t len, uint8_t poly) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= dat[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ poly);
      else            crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// Tesla additive checksum (verified vs 57 real 0x334 frames). NOT a CRC.
//   byte7 = (sum(bytes[0..6]) + (id & 0xFF) + (id >> 8)) & 0xFF
// Call AFTER editing payload; counter (hi nibble of byte6) is left untouched.
static inline void teslaFixChecksum(uint8_t *d, uint16_t id) {
  uint16_t sum = (id & 0xFF) + ((id >> 8) & 0xFF);
  for (uint8_t i = 0; i < 7; i++) sum += d[i];
  d[7] = (uint8_t)(sum & 0xFF);
}

// Cut is active only while a fresh, valid 0x201 says so. Stale/absent -> false
// (= pass 0x334 through unchanged), per the chosen failsafe.
// A 0x201 is "fresh" only within the timeout. Stale/absent -> no edits
// (= pass 0x334 through unchanged), per the chosen failsafe.
static inline bool cut201Fresh() {
  return (millis() - g_last_201_ms) < TORQUE_CUT_TIMEOUT_MS;
}
// Regen must be killed for EITHER flag: braking (no regen) and freeroll (no
// torque of any sign). The drive side of all-torque-cut is handled upstream by
// the VCU/pedal interceptor zeroing the request.
static inline bool regenClampActive() {
  return cut201Fresh() && (g_regen_cut || g_all_cut);
}

// 0x268 ID268SystemPower (5 bytes, NO checksum/counter). Encoding confirmed
// against a real log (01 01 1B 00 73 -> drive 27 kW, regen 15 kW):
//   SystemDrivePowerMax268 : bits 16-24 (byte2 + bit0 of byte3), kW, raw = kW
//   SystemRegenPowerMax268 : byte4, kW = raw - 100  (so 0 kW -> raw 100)
// This is the EFFECTIVE power lever to the drive unit, applied by the T2C after
// its (buggy) rampdown. We overwrite ABSOLUTE values, so we cannot underflow/
// overflow the way the T2C does. Only ever reduces -> bounded-safe.
static inline void modify268(uint8_t *d) {
  const bool fresh = cut201Fresh();
  if (fresh && (g_regen_cut || g_all_cut)) {
    d[4] = 100;            // SystemRegenPowerMax -> 0 kW
  }
  if (fresh && g_all_cut) {
    d[2] = 0x00;          // SystemDrivePowerMax low 8 bits -> 0
    d[3] = (uint8_t)(d[3] & ~0x01);  // SystemDrivePowerMax bit8 -> 0 (preserve other bits)
  }
}

// Copy the diagnostic subset of drive-unit traffic to the VCU bus (Can2),
// renaming 0x108 -> 0x107. Uses a local copy so the relayed frame is never
// mutated. Called for frames seen on BOTH Tesla segments so it is robust to
// which side actually sources a given ID.
static inline void diagCopyToCan2(const CAN_message_t &m) {
  switch (m.id) {
    case CAN_ID_INVERTER_TEMP:  // 0x315
    case CAN_ID_DRIVE_STAT:     // 0x118
    case CAN_ID_REAR_POWER:     // 0x266
    case CAN_ID_MOTOR:          // 0x126
    case CAN_ID_DI_ALERTMATRIX: // 0x35A
    case CAN_ID_DI_LIMITS:      // 0x1D6
      Can2.write(m);
      break;
    case CAN_ID_MOTOR_TORQUE: { // 0x108 -> 0x107
      CAN_message_t c = m;
      c.id = CAN_ID_MOTOR_TORQUE_NEW;
      Can2.write(c);
      break;
    }
    default: break;
  }
}

void initCAN() {
  // CUSTOM POWERTRAIN-CAN (Can2): known bus, selective mailbox RX.
  Can2.begin();
  Can2.setBaudRate(500000);
  Can2.setMBFilter(MB0, CAN_ID_HVREQ);       // HVCU control
  Can2.setMBFilter(MB1, CAN_ID_MAX_POWER);   // -> relay to T2C
  Can2.setMBFilter(MB2, CAN_ID_SHIFT);       // -> relay to T2C
  Can2.setMBFilter(MB3, CAN_ID_TORQUE_CUT);  // 0x201 cut request  [NEW]

  // TESLA control bus - T2C SEGMENT (Can3): accept-all, ordered FIFO.
  // Drained by polling Can3.read() in handleCANMessages(). Do NOT enable the
  // FIFO interrupt: with it set, readFIFO() is polling-blocked and frames only
  // come out via events()/onReceive(), which we don't use.
  Can3.begin();
  Can3.setBaudRate(500000);
  Can3.setMaxMB(16);
  Can3.enableFIFO();
  Can3.setFIFOFilter(ACCEPT_ALL);

  // TESLA control bus - DRIVE-UNIT SEGMENT (Can4): accept-all, ordered FIFO.
  // Polled, same as Can3 - no FIFO interrupt (see note above).
  Can4.begin();
  Can4.setBaudRate(500000);
  Can4.setMaxMB(16);
  Can4.enableFIFO();
  Can4.setFIFOFilter(ACCEPT_ALL);
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
  pinMode(CRUISE_EN, OUTPUT);

  digitalWrite(MCONP_GATE, LOW);
  digitalWrite(MCONN_GATE, LOW);
  digitalWrite(CCSP_GATE, LOW);
  digitalWrite(CCSN_GATE, LOW);
  digitalWrite(PRECHARGE, LOW);
  digitalWrite(CRUISE_EN, LOW);

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
  }

  // Check if we should start or cancel sleep countdown
  if (shouldEnterSleep()) {
    if (!sleepCountdownActive) {
      // Start countdown
      sleepCountdownActive = true;
      sleepCountdownStart = millis();
    } else if (millis() - sleepCountdownStart >= SLEEP_COUNTDOWN_TIME) {
      // Countdown complete, enter sleep
      enterLowPower();
    }
  } else { 
    // Conditions no longer met for sleep, cancel countdown
    if (sleepCountdownActive) {
      cancelSleepCountdown();
    }
  }
}

int8_t getChargeportTemp() {
  uint16_t adc = analogRead(A1);
  uint16_t last = 0;
  for (uint8_t i = 0; i < 71; i++) {
    uint16_t cur = tempSensorTable[i];
    if (cur >= adc) {
      if (i == 0) return 0; // -30°C minimum
      float frac = (float)(cur - adc) / (cur - last);
      float temp = (2.0f * i) - 30.0f - (2.0f * frac);
      return (uint8_t)(temp + 30);
    }
    last = cur;
  }
  return 140;   // +110°C (end of table)
}

bool shouldEnterSleep() {
  // All conditions must be met to enter sleep:
  // - Key is OFF
  // - All contactors are OFF
  return (digitalRead(KEYON_IN) == LOW &&
          mconnState == OFF &&
          mconpState == OFF &&
          ccsnState == OFF &&
          ccspState == OFF);
}

void cancelSleepCountdown() {
  sleepCountdownActive = false;
  sleepCountdownStart = 0;
}

void handleCANMessages() {
  // ---------- CUSTOM POWERTRAIN-CAN (Can2) ----------
  while (Can2.read(rxMsg)) {
    // Relay T2C control inputs from the VCU to the T2C segment.
    if (rxMsg.id == CAN_ID_MAX_POWER || rxMsg.id == CAN_ID_SHIFT) {
      Can3.write(rxMsg);
    }
 
    // Torque-cut request: update state ONLY on a CRC-valid frame.
    if (rxMsg.id == CAN_ID_TORQUE_CUT && rxMsg.len == 3) {
      if (crc8(rxMsg.buf, 2, CAN_CRC_POLY) == rxMsg.buf[2]) {
        g_regen_cut = (rxMsg.buf[0] & REGEN_CUT_FLAG) != 0;
        g_all_cut   = (rxMsg.buf[0] & ALL_TORQUE_CUT_FLAG) != 0;
        g_last_201_ms   = millis();        // only valid frames refresh freshness
      }
      // invalid CRC -> ignore; staleness timeout will fall back to pass-through
    }
 
    // VCU HVCU control: [0x39, precharge, negative, positive]
    if (rxMsg.id == CAN_ID_HVREQ && rxMsg.len == 4 && rxMsg.buf[0] == 0x39) {
      if (rxMsg.buf[1] == 0x02) prechargeEnable = true;
      else if (rxMsg.buf[1] == 0x03) prechargeEnable = false;
      if (rxMsg.buf[2] == 0x02) mconnEnable = true;
      else if (rxMsg.buf[2] == 0x03) mconnEnable = false;
      if (rxMsg.buf[3] == 0x02) mconpEnable = true;
      else if (rxMsg.buf[3] == 0x03) mconpEnable = false;
    }
  }
 
  // ---------- TESLA control: T2C SEGMENT (Can3) -> DRIVE UNIT (Can4) ----------
  // Transparent relay of EVERYTHING. Only 0x334 is touched, only on active cut.
  while (Can3.read(rxMsg)) {
    if (rxMsg.id == CAN_ID_POWERTRAIN_CTRL && rxMsg.len == 8) {
      bool edited = false;
      // Regen clamp: covers braking (regen cut) AND the regen side of freeroll.
      if (regenClampActive()) {
        rxMsg.buf[3] = 0x00;                 // UI_regenTorqueMax -> 0
        edited = true;
      }
#if ENABLE_EXPERIMENTAL_MOTOR_IDLE
      // EXPERIMENTAL, bench-only: on all-torque-cut, push the rear DU toward an
      // idle/no-torque state via mode bits, to emulate true neutral freewheel.
      // motorOnMode (byte4 bits2-3) = FRONT_ONLY(1); stoppingMode (byte5 bits0-1)
      // = STANDARD(0). UNVALIDATED on a front-less car - may fault.
      if (cut201Fresh() && g_all_cut) {
        rxMsg.buf[4] = (uint8_t)((rxMsg.buf[4] & ~(0x3 << 2)) | (0x1 << 2)); // FRONT_ONLY
        rxMsg.buf[5] = (uint8_t)( rxMsg.buf[5] & ~0x3);                      // STANDARD
        edited = true;
      }
#endif
      if (edited) teslaFixChecksum(rxMsg.buf, CAN_ID_POWERTRAIN_CTRL);
      Can2.write(rxMsg); // Copy 0x334 to VCU bus.
    }
    else if (rxMsg.id == CAN_ID_SYSTEM_POWER && rxMsg.len == 5) {
      // 0x268 is the EFFECTIVE power lever (post-rampdown). No checksum to fix.
      modify268(rxMsg.buf);
      Can2.write(rxMsg); // Copy to VCU
    }
    Can4.write(rxMsg);        // forward (modified iff a cut is active)
    
    diagCopyToCan2(rxMsg);    // copy any T2C-sourced diagnostic IDs to the VCU
  }
 
  // ---------- TESLA control: DRIVE UNIT (Can4) -> T2C SEGMENT (Can3) ----------
  // Transparent relay of EVERYTHING, plus diagnostic copy to the VCU bus.
  while (Can4.read(rxMsg)) {
    Can3.write(rxMsg);        // forward untouched
    diagCopyToCan2(rxMsg);    // 0x315/0x118/0x266/0x268/0x35A and 0x108->0x107
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
  txMsg.buf[4] = getChargeportTemp();
  txMsg.len = 5;
  Can2.write(txMsg);
}

void enterLowPower() {
  // Mask + clear the FlexCAN interrupts so KEYON is the only wake source.
  // hibernate() sleeps at a single WFI; a CAN interrupt pending at that instant
  // makes the WFI a no-op and the core hangs instead of powering down.
  NVIC_DISABLE_IRQ(IRQ_CAN1);   // Can4 peripheral
  NVIC_DISABLE_IRQ(IRQ_CAN2);   // Can2 peripheral
  NVIC_DISABLE_IRQ(IRQ_CAN3);   // Can3 peripheral
  NVIC_CLEAR_PENDING(IRQ_CAN1);
  NVIC_CLEAR_PENDING(IRQ_CAN2);
  NVIC_CLEAR_PENDING(IRQ_CAN3);

  // Deep sleep, wake on KEYON rising. The chip resets on wake and re-runs
  // setup(), so execution does not continue past this call.
  Snooze.hibernate(config);
}