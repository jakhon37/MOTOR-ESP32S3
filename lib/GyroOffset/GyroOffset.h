// MOTOR-ESP32S3/lib/GyroOffset/GyroOffset.h
// Keeps the gyro's resting offsets current. They move as the board warms up, so whenever the robot
// stands still they're measured again from the gyro itself. No Arduino dependency, so it can be
// tested on a PC.
#ifndef GYRO_OFFSET_H
#define GYRO_OFFSET_H

#include <stdint.h>

struct GyroOffsetTuning {
    float start[3];           // offsets measured at rest, rad/s, in the IMU's frame
    uint32_t settleMs;        // the robot must be still this long before readings count
    uint16_t windowReadings;  // readings averaged for each update
    int32_t maxCounts;        // a wheel count moving by this much means the robot isn't still
    float maxSd;              // a window is used only if each axis's standard deviation is under this
    float maxShift;           // and each axis's mean is this close to start, rad/s
    float rate;               // the share of the way to a used window's mean that the offsets move
};

GyroOffsetTuning defaultGyroOffsetTuning();  // from config.h

class GyroOffset {
public:
    explicit GyroOffset(const GyroOffsetTuning& tuning);

    // One gyro reading, rad/s in the IMU's frame. driven: either motor gets PWM. The counts are the
    // wheels' running totals.
    void update(const float gyro[3], bool driven, int32_t leftCount, int32_t rightCount, uint32_t nowMs);

    float offset(int axis) const { return m_offset[axis]; }
    uint32_t windowsUsed() const { return m_used; }
    uint32_t windowsRejected() const { return m_rejected; }

private:
    void restartWindow();

    const GyroOffsetTuning& m_tuning;
    float m_offset[3];

    bool m_still = false;
    uint32_t m_stillSinceMs = 0;
    int32_t m_stillLeft = 0;
    int32_t m_stillRight = 0;

    // Sums of the readings' distance from start, which keeps the variance precise in floats.
    uint16_t m_readings = 0;
    float m_sum[3] = {0, 0, 0};
    float m_sumSquares[3] = {0, 0, 0};

    uint32_t m_used = 0;
    uint32_t m_rejected = 0;
};

#endif
