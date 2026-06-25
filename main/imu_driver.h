#pragma once
// LSM6DS3(TR-C) IMU hardware driver for the XIAO nRF52840 Sense.
// Talks to the on-board IMU over Wire1 (TWI1). All register/address details and
// the I2C transport are private to imu_driver.cpp; only the types and the two
// entry points below cross into the application.
#include "imu_types.h"   // defines Vec3 (shared with the detector modules)
#include <cstdint>

// One decoded IMU reading in SI units.
struct ImuSample {
  Vec3 accelMps2;
  Vec3 gyroRad;
  float tempC;
};

// The same reading as raw signed register counts (for CSV logging / replay).
struct RawImuCounts {
  int16_t gx, gy, gz, ax, ay, az;
};

// Defined in imu_driver.cpp; main.ino both reads and writes it (it clears the
// flag when a read fails so the loop drops back into the no-IMU retry path).
extern bool imuReady;

// Power on + probe + configure the IMU. Sets imuReady / the internal address.
// Returns true on success. Does NOT touch any application state machine.
bool beginImu();

// Read one sample. Returns false (and leaves the args untouched) if the IMU is
// not ready or the I2C transaction fails.
bool readImu(ImuSample& sample, RawImuCounts& counts);
