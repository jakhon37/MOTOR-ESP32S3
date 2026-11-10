// MOTOR-ESP32S3/lib/GyroOffset/GyroOffset.cpp
#include "GyroOffset.h"

#include <math.h>
#include "config.h"

GyroOffsetTuning defaultGyroOffsetTuning() {
    GyroOffsetTuning t;
    t.start[0] = GYRO_BIAS_X;
    t.start[1] = GYRO_BIAS_Y;
    t.start[2] = GYRO_BIAS_Z;
    t.settleMs = GYRO_LEARN_SETTLE_MS;
    t.windowReadings = GYRO_LEARN_WINDOW;
    t.maxCounts = GYRO_LEARN_MAX_COUNTS;
    t.maxSd = GYRO_LEARN_MAX_SD;
    t.maxShift = GYRO_LEARN_MAX_SHIFT;
    t.rate = GYRO_LEARN_RATE;
    return t;
}

static int32_t distance(int32_t a, int32_t b) {
    return a > b ? a - b : b - a;
}

GyroOffset::GyroOffset(const GyroOffsetTuning& tuning) : m_tuning(tuning) {
    for (int i = 0; i < 3; i++) {
        m_offset[i] = tuning.start[i];
    }
}

void GyroOffset::update(const float gyro[3], bool driven, int32_t leftCount, int32_t rightCount, uint32_t nowMs) {
    const bool wheelsStill = distance(leftCount, m_stillLeft) < m_tuning.maxCounts &&
                             distance(rightCount, m_stillRight) < m_tuning.maxCounts;
    if (driven || !m_still || !wheelsStill) {
        // Moving, or just stopped: the wait for the robot to settle starts again from here.
        m_still = !driven;
        m_stillSinceMs = nowMs;
        m_stillLeft = leftCount;
        m_stillRight = rightCount;
        restartWindow();
        return;
    }
    if (nowMs - m_stillSinceMs < m_tuning.settleMs) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        const float d = gyro[i] - m_tuning.start[i];
        m_sum[i] += d;
        m_sumSquares[i] += d * d;
    }
    if (++m_readings < m_tuning.windowReadings) {
        return;
    }

    // A hand on the robot shows as spread, a slow turn with the wheels lifted as a mean far from
    // start. Either way the window isn't an offset.
    bool steady = true;
    float mean[3];
    for (int i = 0; i < 3; i++) {
        const float m = m_sum[i] / m_readings;
        const float variance = m_sumSquares[i] / m_readings - m * m;
        steady = steady && variance < m_tuning.maxSd * m_tuning.maxSd && fabsf(m) <= m_tuning.maxShift;
        mean[i] = m_tuning.start[i] + m;
    }
    if (steady) {
        for (int i = 0; i < 3; i++) {
            m_offset[i] += m_tuning.rate * (mean[i] - m_offset[i]);
        }
        m_used++;
    } else {
        m_rejected++;
    }
    restartWindow();
}

void GyroOffset::restartWindow() {
    m_readings = 0;
    for (int i = 0; i < 3; i++) {
        m_sum[i] = 0;
        m_sumSquares[i] = 0;
    }
}
