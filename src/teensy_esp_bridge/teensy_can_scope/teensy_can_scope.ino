#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <string.h>

// Oscilloscope capture settings
constexpr uint32_t kCanBitrate = 500000;
constexpr uint32_t kCanId = 0x123;
constexpr uint32_t kFramePeriodUs = 20000;  // 20 ms (50 frames/s)
constexpr uint32_t kStatusPeriodUs = 1000000;
constexpr FLEXCAN_MAILBOX kTxMailbox = MB8;

// CAN1 sends the fixed frame. CAN2 is a second active CAN node that supplies
// the ACK bit, so this test does not require a USB-CAN adapter or another MCU.
// Each CAN controller must have its own transceiver.
FlexCAN_T4<CAN1, RX_SIZE_16, TX_SIZE_16> txCan;                                                                                                                                                                                                                                                                                                                              ***********
*+/
                              --AN_T4<CAN2, RX_SIZE_16, TX_SIZE_16> ackCan;

volatile bool txInFlight = false;
volatile uint32_t txCompleteCount = 0;

uint32_t nextFrameUs = 0;
uint32_t nextStatusUs = 0;
uint32_t rxCount = 0;
uint32_t rxMismatchCount = 0;

static bool timeReached(uint32_t now, uint32_t deadline)
{
  return static_cast<int32_t>(now - deadline) >= 0;
}

static CAN_message_t makeFixedFrame()
{
  CAN_message_t frame = {};
  frame.id = kCanId;
  frame.len = 8;
  frame.flags.extended = false;

  // This payload never changes. Keeping the complete CAN frame identical also
  // keeps arbitration, data, CRC and bit-stuffing positions identical.
  const uint8_t payload[8] = {
      0x55, 0xAA, 0x00, 0xFF, 0x12, 0x34, 0x56, 0x78,
  };
  memcpy(frame.buf, payload, sizeof(payload));
  return frame;
}

static bool isExpectedFrame(const CAN_message_t &frame)
{
  const uint8_t expected[8] = {
      0x55, 0xAA, 0x00, 0xFF, 0x12, 0x34, 0x56, 0x78,
  };
  return frame.id == kCanId && frame.len == sizeof(expected) &&
         memcmp(frame.buf, expected, sizeof(expected)) == 0;
}

void transmitComplete(const CAN_message_t &message)
{
  (void)message;
  ++txCompleteCount;
  txInFlight = false;
}

static void initializeCan()
{
  txCan.begin();
  txCan.setBaudRate(kCanBitrate);
  txCan.setMaxMB(16);
  txCan.setMB(kTxMailbox, TX, STD);
  txCan.enableMBInterrupt(kTxMailbox);
  txCan.onTransmit(kTxMailbox, transmitComplete);

  // CAN2 only receives/acknowledges the CAN1 frame. Reading it in loop() keeps
  // the receive mailbox clear during a long oscilloscope session.
  ackCan.begin();
  ackCan.setBaudRate(kCanBitrate);
  ackCan.setMaxMB(16);
  ackCan.setMB(MB8, RX, STD);
  ackCan.setMBFilter(MB8, kCanId);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {
  }

  initializeCan();
  const uint32_t now = micros();
  nextFrameUs = now + 100000;
  nextStatusUs = now + kStatusPeriodUs;

  Serial.println("Teensy fixed CAN oscilloscope source started");
  Serial.printf("CAN1 TX pin=22 RX pin=23, CAN2 ACK TX pin=1 RX pin=0\n");
  Serial.printf("bitrate=%lu id=0x%03lX period_us=%lu payload=55 AA 00 FF 12 34 56 78\n",
                static_cast<unsigned long>(kCanBitrate),
                static_cast<unsigned long>(kCanId),
                static_cast<unsigned long>(kFramePeriodUs));
}

void loop()
{
  const uint32_t now = micros();

  if (timeReached(now, nextFrameUs)) {
    // Preserve the absolute 20 ms schedule even if loop() was briefly delayed.
    do {
      nextFrameUs += kFramePeriodUs;
    } while (timeReached(now, nextFrameUs));

    if (!txInFlight) {
      const CAN_message_t frame = makeFixedFrame();
      txInFlight = true;
      if (txCan.write(kTxMailbox, frame) != 1) {
        txInFlight = false;
      }
    }
  }

  CAN_message_t received;
  while (ackCan.read(received)) {
    ++rxCount;
    if (!isExpectedFrame(received)) {
      ++rxMismatchCount;
    }
  }

  if (timeReached(now, nextStatusUs)) {
    do {
      nextStatusUs += kStatusPeriodUs;
    } while (timeReached(now, nextStatusUs));

    uint32_t completed;
    noInterrupts();
    completed = txCompleteCount;
    interrupts();

    Serial.printf("STAT tx_complete=%lu rx=%lu mismatch=%lu in_flight=%u %s\n",
                  static_cast<unsigned long>(completed),
                  static_cast<unsigned long>(rxCount),
                  static_cast<unsigned long>(rxMismatchCount),
                  txInFlight ? 1U : 0U,
                  (completed == rxCount && rxMismatchCount == 0) ? "OK" :
                                                                  "CHECK");
  }
}
