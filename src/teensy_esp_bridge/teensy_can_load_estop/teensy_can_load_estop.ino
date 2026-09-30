#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <string.h>

#include "config.h"

FlexCAN_T4<CAN1, RX_SIZE_16, TX_SIZE_16> dumpCan;
constexpr FLEXCAN_MAILBOX kDumpMailbox[2] = {MB8, MB9};

struct Statistics {
  uint32_t generated = 0;
  uint32_t enqueued = 0;
  uint32_t dropped = 0;
  uint32_t mailboxBusy = 0;
  uint32_t txTimeout = 0;
  uint32_t txUnconfirmed = 0;
  uint32_t recoveries = 0;
  uint32_t canErrors = 0;
  uint32_t busOff = 0;
  uint32_t debouncedEdges = 0;
} stats;

volatile uint8_t rawLevel = HIGH;
volatile uint32_t rawEdgeUs = 0;
volatile uint32_t rawEdges = 0;
volatile bool rawChanged = false;
uint8_t candidateLevel = HIGH;
uint8_t debouncedLevel = HIGH;
uint32_t candidateSinceUs = 0;
bool debouncePending = true;
bool switchInitialized = false;

volatile bool loadOn = false;
volatile uint32_t loadCycle = 0;
volatile uint32_t lastOnUs = 0;
uint32_t lastOffUs = 0;
uint32_t nextGenerationUs = 0;
uint32_t sequence = 0;
CAN_message_t latestFrame;
bool latestPending = false;

volatile bool inFlight[2] = {false, false};
volatile uint32_t mailboxCycle[2] = {0, 0};
volatile uint32_t requestUs[2] = {0, 0};
volatile uint32_t txSuccess = 0;
volatile uint32_t lastCompleteUs = 0;
volatile bool firstPending = false;
volatile bool firstReady = false;
volatile uint32_t firstCycle = 0;
volatile uint32_t firstLatencyUs = 0;

volatile bool drainActive = false;
volatile uint32_t drainCycle = 0;
volatile uint32_t drainOffUs = 0;
volatile uint32_t drainLastUs = 0;
volatile uint8_t drainInitial = 0;
volatile uint8_t drainRemaining = 0;
volatile uint8_t drainCompleted = 0;
volatile uint8_t drainUnconfirmed = 0;

uint32_t nextStatsUs = 0;
bool wasBusOff = false;
bool recoveryPending = false;
uint32_t nextRecoveryUs = 0;

static bool reached(uint32_t now, uint32_t deadline)
{
  return static_cast<int32_t>(now - deadline) >= 0;
}

static void writeBe32(uint8_t *out, uint32_t value)
{
  out[0] = static_cast<uint8_t>(value >> 24);
  out[1] = static_cast<uint8_t>(value >> 16);
  out[2] = static_cast<uint8_t>(value >> 8);
  out[3] = static_cast<uint8_t>(value);
}

void switchEdgeIsr()
{
  rawLevel = static_cast<uint8_t>(digitalRead(kLoadSwitchPin));
  rawEdgeUs = micros();
  ++rawEdges;
  rawChanged = true;
}

static void dumpTransmitComplete(const CAN_message_t &message)
{
  const int index = message.mb == MB8 ? 0 : message.mb == MB9 ? 1 : -1;
  if (index < 0) return;
  const uint32_t now = micros();
  const uint32_t cycle = mailboxCycle[index];
  inFlight[index] = false;
  ++txSuccess;
  lastCompleteUs = now;
  if (firstPending && cycle == loadCycle) {
    firstPending = false;
    firstReady = true;
    firstCycle = cycle;
    firstLatencyUs = now - lastOnUs;
  }
  if (drainActive && cycle == drainCycle && drainRemaining > 0) {
    --drainRemaining;
    ++drainCompleted;
    drainLastUs = now;
  }
}

static void initializeCan()
{
  dumpCan.begin();
  dumpCan.setBaudRate(kCanBitrate);
  dumpCan.setMaxMB(16);
  for (int index = 0; index < 2; ++index) {
    dumpCan.setMB(kDumpMailbox[index], TX, STD);
    dumpCan.enableMBInterrupt(kDumpMailbox[index]);
    dumpCan.onTransmit(kDumpMailbox[index], dumpTransmitComplete);
  }
}

static void processSwitch(uint32_t now)
{
  noInterrupts();
  const bool changed = rawChanged;
  const uint8_t level = rawLevel;
  const uint32_t edgeUs = rawEdgeUs;
  rawChanged = false;
  interrupts();

  if (changed) {
    candidateLevel = level;
    candidateSinceUs = edgeUs;
    debouncePending = true;
  }
  if (!debouncePending ||
      static_cast<uint32_t>(now - candidateSinceUs) < kDebounceUs) return;
  debouncePending = false;
  if (switchInitialized && candidateLevel == debouncedLevel) return;

  const bool wasInitialized = switchInitialized;
  switchInitialized = true;
  debouncedLevel = candidateLevel;
  if (wasInitialized) ++stats.debouncedEdges;

  if (candidateLevel == kLoadActiveLevel) {
    noInterrupts();
    loadOn = true;
    ++loadCycle;
    lastOnUs = now;
    firstPending = true;
    firstReady = false;
    interrupts();
    latestPending = false;
    nextGenerationUs = now;
    Serial.printf("LOAD on cycle=%lu at_us=%lu\n",
                  static_cast<unsigned long>(loadCycle),
                  static_cast<unsigned long>(now));
  } else if (wasInitialized) {
    noInterrupts();
    loadOn = false;
    lastOffUs = now;
    drainActive = true;
    drainCycle = loadCycle;
    drainOffUs = now;
    drainLastUs = now;
    drainInitial = static_cast<uint8_t>(inFlight[0]) +
                   static_cast<uint8_t>(inFlight[1]);
    drainRemaining = drainInitial;
    drainCompleted = 0;
    drainUnconfirmed = 0;
    interrupts();
    if (latestPending) ++stats.dropped;
    latestPending = false;
    Serial.printf("LOAD off cycle=%lu at_us=%lu pending_mb=%u\n",
                  static_cast<unsigned long>(loadCycle),
                  static_cast<unsigned long>(now),
                  static_cast<unsigned int>(drainInitial));
  } else {
    Serial.printf("LOAD boot_off at_us=%lu\n",
                  static_cast<unsigned long>(now));
  }
}

static void collectCanErrors(uint32_t now)
{
  CAN_error_t error;
  while (dumpCan.error(error, false)) {
    if (error.BIT1_ERR || error.BIT0_ERR || error.ACK_ERR ||
        error.CRC_ERR || error.FRM_ERR || error.STF_ERR ||
        error.TX_WRN || error.RX_WRN) ++stats.canErrors;
    const bool busOff = strcmp(error.FLT_CONF, "Bus off") == 0;
    if (busOff && !wasBusOff) {
      ++stats.busOff;
      recoveryPending = true;
      nextRecoveryUs = now + kCanRecoveryBackoffUs;
    }
    wasBusOff = busOff;
  }
}

static void checkTransmitTimeout(uint32_t now)
{
  if (recoveryPending) return;
  for (int index = 0; index < 2; ++index) {
    if (inFlight[index] &&
        static_cast<uint32_t>(now - requestUs[index]) >= kDumpTxTimeoutUs) {
      ++stats.txTimeout;
      recoveryPending = true;
      nextRecoveryUs = now + kCanRecoveryBackoffUs;
      return;
    }
  }
}

static void recoverCan(uint32_t now)
{
  if (!recoveryPending || !reached(now, nextRecoveryUs)) return;
  // FlexCAN_T4::write(mb, frame) can use its internal TX queue when busy.
  // There is no public API to discard that queue. Never reset with queued
  // frames that could be transmitted after the switch has been released.
  if (dumpCan.getTXQueueCount() != 0) {
    nextRecoveryUs = now + kCanRecoveryBackoffUs;
    return;
  }
  noInterrupts();
  for (int index = 0; index < 2; ++index) {
    if (!inFlight[index]) continue;
    ++stats.txUnconfirmed;
    if (drainActive && mailboxCycle[index] == drainCycle &&
        drainRemaining > 0) {
      --drainRemaining;
      ++drainUnconfirmed;
    }
    inFlight[index] = false;
  }
  initializeCan();  // begin() resets the controller; restore baud and mailboxes.
  interrupts();
  if (latestPending) ++stats.dropped;
  latestPending = false;
  nextGenerationUs = now;
  wasBusOff = false;
  recoveryPending = false;
  ++stats.recoveries;
  Serial.printf("CAN_RECOVERY at_us=%lu unconfirmed_total=%lu\n",
                static_cast<unsigned long>(now),
                static_cast<unsigned long>(stats.txUnconfirmed));
}

static void generateLatestDump(uint32_t now)
{
  if (!loadOn || recoveryPending || !reached(now, nextGenerationUs)) return;
  const uint32_t count = (now - nextGenerationUs) / kDumpPeriodUs + 1;
  nextGenerationUs += count * kDumpPeriodUs;
  stats.generated += count;
  stats.dropped += count - 1;
  if (latestPending) ++stats.dropped;
  sequence += count;
  latestFrame = CAN_message_t{};
  latestFrame.id = kLoadCanId;
  latestFrame.len = 8;
  latestFrame.flags.extended = false;
  writeBe32(&latestFrame.buf[0], sequence - 1);
  writeBe32(&latestFrame.buf[4], now);
  latestPending = true;
}

static void enqueueLatestDump(uint32_t now)
{
  if (!loadOn || recoveryPending || !latestPending) return;
  const int index = !inFlight[0] ? 0 : !inFlight[1] ? 1 : -1;
  if (index < 0) {
    ++stats.mailboxBusy;
    return;
  }
  noInterrupts();
  // FlexCAN_T4::write() queues a frame if the requested hardware mailbox is
  // busy. Check the installed library's mailbox code before calling it so an
  // OFF transition cannot leave a hidden software-queued frame behind.
  if (FLEXCAN_get_code(FLEXCANb_MBn_CS(CAN1, kDumpMailbox[index])) !=
      FLEXCAN_MB_CODE_TX_INACTIVE) {
    interrupts();
    ++stats.mailboxBusy;
    recoveryPending = true;
    nextRecoveryUs = now + kCanRecoveryBackoffUs;
    return;
  }
  inFlight[index] = true;
  mailboxCycle[index] = loadCycle;
  requestUs[index] = now;
  const int accepted = dumpCan.write(kDumpMailbox[index], latestFrame);
  if (accepted != 1) inFlight[index] = false;
  interrupts();
  if (accepted == 1) {
    latestPending = false;
    ++stats.enqueued;
  } else {
    ++stats.mailboxBusy;
  }
}

static void printDrain()
{
  if (!drainActive || drainRemaining != 0) return;
  const uint32_t latency = drainCompleted == 0 ? 0 :
                           drainLastUs - drainOffUs;
  Serial.printf(
      "DRAIN cycle=%lu initial=%u completed=%u unconfirmed=%u "
      "last_complete_us=%lu drain_us=%lu\n",
      static_cast<unsigned long>(drainCycle),
      static_cast<unsigned int>(drainInitial),
      static_cast<unsigned int>(drainCompleted),
      static_cast<unsigned int>(drainUnconfirmed),
      static_cast<unsigned long>(drainCompleted == 0 ? 0 : drainLastUs),
      static_cast<unsigned long>(latency));
  drainActive = false;
}

static void printStatistics(uint32_t now)
{
  if (!reached(now, nextStatsUs)) return;
  nextStatsUs += kStatsPeriodUs;
  Serial.printf(
      "STAT load_switch=%s raw_edges=%lu debounced_edges=%lu "
      "last_on_us=%lu last_off_us=%lu dump_generated=%lu "
      "dump_enqueued=%lu dump_tx_success=%lu dump_dropped=%lu "
      "dump_mailbox_busy=%lu dump_in_flight=%u dump_tx_timeout=%lu "
      "dump_tx_unconfirmed=%lu can_recoveries=%lu can_tx_queue=%lu "
      "can_error_count=%lu bus_off_count=%lu last_tx_complete_us=%lu\n",
      loadOn ? "on" : "off",
      static_cast<unsigned long>(rawEdges),
      static_cast<unsigned long>(stats.debouncedEdges),
      static_cast<unsigned long>(lastOnUs),
      static_cast<unsigned long>(lastOffUs),
      static_cast<unsigned long>(stats.generated),
      static_cast<unsigned long>(stats.enqueued),
      static_cast<unsigned long>(txSuccess),
      static_cast<unsigned long>(stats.dropped),
      static_cast<unsigned long>(stats.mailboxBusy),
      static_cast<unsigned int>(inFlight[0]) +
          static_cast<unsigned int>(inFlight[1]),
      static_cast<unsigned long>(stats.txTimeout),
      static_cast<unsigned long>(stats.txUnconfirmed),
      static_cast<unsigned long>(stats.recoveries),
      static_cast<unsigned long>(dumpCan.getTXQueueCount()),
      static_cast<unsigned long>(stats.canErrors),
      static_cast<unsigned long>(stats.busOff),
      static_cast<unsigned long>(lastCompleteUs));
}

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  pinMode(kLoadSwitchPin, INPUT_PULLUP);
  const uint32_t now = micros();
  const uint8_t initialLevel = static_cast<uint8_t>(digitalRead(kLoadSwitchPin));
  rawLevel = initialLevel;
  rawEdgeUs = now;
  candidateLevel = initialLevel;
  candidateSinceUs = now;
  initializeCan();
  attachInterrupt(digitalPinToInterrupt(kLoadSwitchPin), switchEdgeIsr, CHANGE);
  nextStatsUs = now + kStatsPeriodUs;
  Serial.println("Teensy 4.1 switch-gated CAN1 load test started");
  Serial.printf("bitrate=%lu load_id=0x%03lX dump_period_us=%lu "
                "switch_pin=%u active=LOW debounce_us=%lu\n",
                static_cast<unsigned long>(kCanBitrate),
                static_cast<unsigned long>(kLoadCanId),
                static_cast<unsigned long>(kDumpPeriodUs),
                static_cast<unsigned int>(kLoadSwitchPin),
                static_cast<unsigned long>(kDebounceUs));
}

void loop()
{
  const uint32_t now = micros();
  processSwitch(now);
  collectCanErrors(now);
  checkTransmitTimeout(now);
  recoverCan(now);
  generateLatestDump(now);
  enqueueLatestDump(now);
  printDrain();
  if (firstReady) {
    noInterrupts();
    const uint32_t cycle = firstCycle;
    const uint32_t latency = firstLatencyUs;
    firstReady = false;
    interrupts();
    Serial.printf("FIRST_TX cycle=%lu on_to_complete_us=%lu\n",
                  static_cast<unsigned long>(cycle),
                  static_cast<unsigned long>(latency));
  }
  printStatistics(now);
}
