#pragma once

#include <Zumo32U4.h>

class ZumoMotion {
public:
  explicit ZumoMotion(Zumo32U4IMU &imu) : imu_(imu) {}

  bool calibrateGyro();
  void resetHeading();
  bool updateHeading();
  int64_t headingMicrodegrees() const;
  bool rotate(int16_t degrees, int16_t speed, Zumo32U4LineSensors *sensors = nullptr);
  bool calibrateLineSensors(Zumo32U4LineSensors &sensors);

private:
  Zumo32U4IMU &imu_;
  int32_t bias_ = 0;
  uint32_t lastSampleMicros_ = 0;
  int64_t integratedRate_ = 0;
};
