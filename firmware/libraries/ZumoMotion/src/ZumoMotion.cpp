#include "ZumoMotion.h"

namespace {
constexpr uint16_t gyroSamples = 1024;
constexpr uint32_t gyroTimeoutMs = 3000;
constexpr uint32_t rotationTimeoutMs = 6000;
constexpr int16_t calibrationSpeed = 160;
}

bool ZumoMotion::calibrateGyro() {
  if (!imu_.init()) return false;
  imu_.enableDefault();
  if (imu_.getLastError()) return false;
  imu_.configureForTurnSensing();
  if (imu_.getLastError()) return false;

  delay(500);
  const uint32_t start = millis();
  int32_t total = 0;
  uint16_t count = 0;
  while (count < gyroSamples && millis() - start < gyroTimeoutMs) {
    const bool ready = imu_.gyroDataReady();
    if (imu_.getLastError()) return false;
    if (!ready) continue;

    imu_.readGyro();
    if (imu_.getLastError()) return false;
    total += imu_.g.z;
    ++count;
  }
  if (count != gyroSamples) return false;

  bias_ = total / gyroSamples;
  resetHeading();
  return true;
}

void ZumoMotion::resetHeading() {
  integratedRate_ = 0;
  lastSampleMicros_ = micros();
}

bool ZumoMotion::updateHeading() {
  imu_.readGyro();
  if (imu_.getLastError()) return false;

  const uint32_t now = micros();
  const uint32_t elapsed = now - lastSampleMicros_;
  lastSampleMicros_ = now;
  integratedRate_ += static_cast<int64_t>(imu_.g.z - bias_) * elapsed;
  return true;
}

int64_t ZumoMotion::headingMicrodegrees() const {
  // Conserve le facteur de conversion effectif de la version du binôme.
  return integratedRate_ * 175 / 10000;
}

bool ZumoMotion::rotate(int16_t degrees, int16_t speed, Zumo32U4LineSensors *sensors) {
  if (degrees == 0) return true;

  const int8_t direction = degrees > 0 ? 1 : -1;
  const int32_t target = static_cast<int32_t>(degrees) * 1000000L;
  const uint32_t start = millis();
  resetHeading();
  Zumo32U4Motors::setSpeeds(-direction * speed, direction * speed);

  bool reachedTarget = false;
  while (millis() - start < rotationTimeoutMs && updateHeading()) {
    if (direction * (headingMicrodegrees() - target) >= 0) {
      reachedTarget = true;
      break;
    }
    if (sensors) sensors->calibrate();
  }
  Zumo32U4Motors::setSpeeds(0, 0);
  return reachedTarget;
}

bool ZumoMotion::calibrateLineSensors(Zumo32U4LineSensors &sensors) {
  const int16_t sweep[] = {90, -180, 90};
  sensors.initFiveSensors();
  delay(100);
  sensors.calibrate();
  for (int16_t degrees : sweep) {
    if (!rotate(degrees, calibrationSpeed, &sensors)) return false;
  }
  return true;
}
