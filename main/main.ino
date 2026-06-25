/*
  PuttIQ putting sensor prototype for Seeed XIAO nRF52840 Sense.

  Serial Monitor: 115200 baud

  Output:
    READY_TO_PUTT
    SWING_STARTED
    PUTT_DETECTED or PUTT_REJECTED

  This file is now thin setup()/loop() wiring only. The application FSM,
  calibration, detection and app<->UI policy live in app_fsm.{h,cpp}; the LSM6DS3
  driver in imu_driver.{h,cpp}; the LVGL pipeline + shared UI data in ui.{h,cpp}.
*/

#include <Arduino.h>
#include <SPI.h>
#include "driver.h"
#include <TFT_eSPI.h>     // TFT_WIDTH/TFT_HEIGHT/TFT_BL macros for printDisplayConfig().
                          // The `tft` instance + LVGL pipeline now live in ui.cpp.
#include "ui.h"          // on-device LVGL pipeline (ui_init/apply/ready, lv_timer_handler)
#include "imu_driver.h"  // LSM6DS3 IMU driver (ImuSample/RawImuCounts, readImu, imuReady)
#include "app_fsm.h"     // application FSM + detection glue (app_init/update/...)

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

static const uint32_t SERIAL_BAUD = 115200;
static const char FW_VERSION[] = "face_zero_start_v71";

// Display config print stays here because it reads TFT_* macros from TFT_eSPI.
static void printDisplayConfig() {
  Serial.print(F("DISPLAY,setup_id="));
#if defined(USER_SETUP_ID)
  Serial.print(USER_SETUP_ID);
#else
  Serial.print(F("unknown"));
#endif
  Serial.print(F(",driver=0x"));
#if defined(TFT_DRIVER)
  Serial.print(TFT_DRIVER, HEX);
#else
  Serial.print(F("unknown"));
#endif
  Serial.print(F(",width="));
  Serial.print(TFT_WIDTH);
  Serial.print(F(",height="));
  Serial.print(TFT_HEIGHT);
  Serial.print(F(",bl="));
  Serial.println(TFT_BL);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);
  uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 300) {
  }

  Serial.print(F("PUTTIQ_FW,version="));
  Serial.println(FW_VERSION);
  printDisplayConfig();

  ui_init();    // display + touch + LVGL (builds initial home from g_uiHomeAuto)
  app_init();   // FSM/detector init + install ui_click_fn (after ui_init)

  Serial.println(F("SETUP,stage=begin_imu"));
  app_start_imu_session();
  Serial.println(imuReady ? F("SETUP,stage=imu_ready") : F("SETUP,stage=no_imu"));
}

void loop() {
  uint32_t nowMs = millis();

  // LVGL milestone: drive the UI every iteration (cheap when idle). Must run
  // before the IMU sample-rate gate below so the panel keeps refreshing and
  // touch stays responsive.
  if (ui_ready()) {
    lv_timer_handler();
  }

  app_report_on_serial_connect(nowMs);

  if (!imuReady) {
    app_no_imu_tick(nowMs);
    return;
  }

  uint32_t nowUs = micros();
  if (!app_sample_due(nowUs)) {
    return;
  }

  ImuSample sample;
  RawImuCounts rawCounts;
  if (!readImu(sample, rawCounts)) {
    app_on_imu_read_failed();
    return;
  }

  app_update(nowMs, nowUs, sample, rawCounts);
}
