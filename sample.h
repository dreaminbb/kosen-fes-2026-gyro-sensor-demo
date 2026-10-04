#ifndef KOSEN_FES_SAMPLE_H
#define KOSEN_FES_SAMPLE_H

#include <stdint.h>

struct SensorData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

constexpr uint16_t RIGHT_TURN_SAMPLE_INTERVAL_MS = 100;

// Synthetic right turn around +Z: 120 deg/s for 30 samples at 100 ms/sample
// gives 360 degrees in 3 seconds. Acceleration is gravity-only (in g).
// Reverse gyroZ's sign if the board's axes define right as negative Z.
static const SensorData rightTurn360Samples[] = {
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 120.0f},
    {100.0f, 10.0f, 1.0f, 0.0f, 0.0f, 120.0f},
};

constexpr uint16_t SAMPLE_DATA_LENGTH =
    sizeof(rightTurn360Samples) / sizeof(rightTurn360Samples[0]);

#endif // KOSEN_FES_SAMPLE_H
