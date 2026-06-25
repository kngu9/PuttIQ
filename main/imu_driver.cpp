// LSM6DS3(TR-C) IMU hardware driver. Moved verbatim out of main.ino (Refactor
// Pass 2, step 1) — behavior is identical; only the location changed. The
// application's state-machine resets that used to live inside beginImu() now
// live at the call sites in main.ino (this driver is hardware-only).
#include "imu_driver.h"
#include <Arduino.h>  // pinMode/digitalWrite/delay + nRF GPIO macros
#include <Wire.h>     // Wire1 (TWI1)

// ---- Register map / scale constants (private to the driver) ----------------
static const uint8_t LSM6DS3_ADDR_6A = 0x6A;
static const uint8_t LSM6DS3_ADDR_6B = 0x6B;
static const uint8_t LSM6DS3_WHO_AM_I = 0x0F;
static const uint8_t LSM6DS3_WHO_AM_I_VALUE = 0x69;
static const uint8_t LSM6DS3C_WHO_AM_I_VALUE = 0x6A;
static const uint8_t LSM6DS3_CTRL1_XL = 0x10;
static const uint8_t LSM6DS3_CTRL2_G = 0x11;
static const uint8_t LSM6DS3_CTRL3_C = 0x12;
static const uint8_t LSM6DS3_OUT_TEMP_L = 0x20;

static const float ACCEL_4G_G_PER_LSB = 0.000122f;
static const float GYRO_500DPS_DPS_PER_LSB = 0.0175f;
static const float DEG_TO_RAD_F = 0.0174532925f;

// ---- Driver state (private) ------------------------------------------------
bool imuReady = false;                       // public via imu_driver.h
static uint8_t imuAddress = 0;
static float imuTempSensitivity = 256.0f;

// ---- Power rail ------------------------------------------------------------
static void setImuPower(bool enabled) {
#if defined(PIN_LSM6DS3TR_C_POWER)
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, enabled ? HIGH : LOW);
#endif

#if defined(NRF_P1)
  NRF_P1->PIN_CNF[8] = ((uint32_t)NRF_GPIO_PIN_DIR_OUTPUT << GPIO_PIN_CNF_DIR_Pos)
                       | ((uint32_t)NRF_GPIO_PIN_INPUT_DISCONNECT << GPIO_PIN_CNF_INPUT_Pos)
                       | ((uint32_t)NRF_GPIO_PIN_NOPULL << GPIO_PIN_CNF_PULL_Pos)
                       | ((uint32_t)NRF_GPIO_PIN_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
                       | ((uint32_t)NRF_GPIO_PIN_NOSENSE << GPIO_PIN_CNF_SENSE_Pos);
  if (enabled) {
    NRF_P1->OUTSET = (1UL << 8);
  } else {
    NRF_P1->OUTCLR = (1UL << 8);
  }
#endif

  delay(50);
}

// ---- I2C register helpers --------------------------------------------------
static bool writeImuReg(uint8_t reg, uint8_t value) {
  if (imuAddress == 0) {
    return false;
  }

  Wire1.beginTransmission(imuAddress);
  Wire1.write(reg);
  Wire1.write(value);
  return Wire1.endTransmission() == 0;
}

static bool readImuRegAt(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) {
    return false;
  }

  if (Wire1.requestFrom(address, (uint8_t)1) != 1) {
    return false;
  }

  value = Wire1.read();
  return true;
}

static bool readImuBytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire1.beginTransmission(imuAddress);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) {
    return false;
  }

  if (Wire1.requestFrom(imuAddress, length) != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire1.read();
  }
  return true;
}

static int16_t i16le(const uint8_t *b) {
  return (int16_t)((uint16_t)b[1] << 8 | b[0]);
}

// ---- Init / read -----------------------------------------------------------
static bool beginImuAt(uint8_t address) {
  uint8_t whoAmI = 0;
  if (!readImuRegAt(address, LSM6DS3_WHO_AM_I, whoAmI)) {
    return false;
  }

  if (whoAmI != LSM6DS3_WHO_AM_I_VALUE && whoAmI != LSM6DS3C_WHO_AM_I_VALUE) {
    return false;
  }

  imuAddress = address;
  imuTempSensitivity = whoAmI == LSM6DS3C_WHO_AM_I_VALUE ? 256.0f : 16.0f;

  // CTRL3_C: BDU + auto-increment. CTRL1/2: 208 Hz, +/-4g, +/-500 dps.
  if (!writeImuReg(LSM6DS3_CTRL3_C, 0x44) ||
      !writeImuReg(LSM6DS3_CTRL1_XL, 0x58) ||
      !writeImuReg(LSM6DS3_CTRL2_G, 0x54)) {
    imuAddress = 0;
    return false;
  }

  return true;
}

bool beginImu() {
  setImuPower(true);
  Wire1.begin();

  if (!beginImuAt(LSM6DS3_ADDR_6A) && !beginImuAt(LSM6DS3_ADDR_6B)) {
    imuReady = false;
    imuAddress = 0;
    return false;
  }

  imuReady = true;
  return true;
}

bool readImu(ImuSample &sample, RawImuCounts &counts) {
  uint8_t b[14];
  if (!imuReady || imuAddress == 0 || !readImuBytes(LSM6DS3_OUT_TEMP_L, b, sizeof(b))) {
    return false;
  }

  int16_t rawTemp = i16le(&b[0]);
  int16_t rawGx = i16le(&b[2]);
  int16_t rawGy = i16le(&b[4]);
  int16_t rawGz = i16le(&b[6]);
  int16_t rawAx = i16le(&b[8]);
  int16_t rawAy = i16le(&b[10]);
  int16_t rawAz = i16le(&b[12]);

  counts.gx = rawGx;
  counts.gy = rawGy;
  counts.gz = rawGz;
  counts.ax = rawAx;
  counts.ay = rawAy;
  counts.az = rawAz;

  sample.tempC = (float)rawTemp / imuTempSensitivity + 25.0f;
  sample.gyroRad.x = rawGx * GYRO_500DPS_DPS_PER_LSB * DEG_TO_RAD_F;
  sample.gyroRad.y = rawGy * GYRO_500DPS_DPS_PER_LSB * DEG_TO_RAD_F;
  sample.gyroRad.z = rawGz * GYRO_500DPS_DPS_PER_LSB * DEG_TO_RAD_F;
  sample.accelMps2.x = rawAx * ACCEL_4G_G_PER_LSB * 9.80665f;
  sample.accelMps2.y = rawAy * ACCEL_4G_G_PER_LSB * 9.80665f;
  sample.accelMps2.z = rawAz * ACCEL_4G_G_PER_LSB * 9.80665f;
  return true;
}
