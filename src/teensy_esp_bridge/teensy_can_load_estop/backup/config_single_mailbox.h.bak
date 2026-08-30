#pragma once

#include <Arduino.h>

// Teensy 4.1 switch input. The switch connects pin 2 to GND when active.
constexpr uint8_t kEstopSwitchPin = 2;
constexpr uint8_t kEstopActiveLevel = LOW;
constexpr uint8_t kEstopInputMode = INPUT_PULLUP;

constexpr uint32_t kCanBitrate = 500000;
constexpr uint32_t kDebounceUs = 50000;
constexpr uint32_t kDumpPeriodUs = 200;
constexpr uint32_t kStatsPeriodUs = 1000000;

constexpr uint8_t kEstopRepeatCount = 3;
constexpr uint32_t kEstopRepeatIntervalUs = 1000;

// false: E-stop=0x001, dump=0x700
// true:  E-stop=0x700, dump=0x001 (intentional priority inversion test)
constexpr bool kPriorityInverted =  false;

// Disable the load generator for the E-stop-only validation stage.
constexpr bool kDumpEnabled = true;

constexpr uint32_t kNormalEstopId = 0x001;
constexpr uint32_t kNormalDumpId = 0x700;
constexpr uint8_t kEstopQueueCapacity = 64;
