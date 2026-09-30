#pragma once

#include <Arduino.h>

// A normally-open test switch connects pin 2 to GND while pressed.
constexpr uint8_t kLoadSwitchPin = 2;
constexpr uint8_t kLoadActiveLevel = LOW;

constexpr uint32_t kCanBitrate = 500000;
constexpr uint32_t kLoadCanId = 0x001;
constexpr uint32_t kDebounceUs = 50000;
constexpr uint32_t kDumpPeriodUs = 200;
constexpr uint32_t kStatsPeriodUs = 1000000;
constexpr uint32_t kDumpTxTimeoutUs = 100000;
constexpr uint32_t kCanRecoveryBackoffUs = 250000;
