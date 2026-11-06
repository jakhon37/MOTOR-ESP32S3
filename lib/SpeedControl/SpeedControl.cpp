// MOTOR-ESP32S3/lib/SpeedControl/SpeedControl.cpp
#include "SpeedControl.h"

#include <math.h>
#include <stdlib.h>

static const float DT_SECONDS = CONTROL_PERIOD_MS / 1000.0f;
static const float PI_F = 3.14159265f;

ControlTuning defaultTuning() {
    ControlTuning t;
    t.left = {FF_DEADBAND_LEFT, FF_SLOPE_LEFT, KICK_PWM_LEFT};
    t.right = {FF_DEADBAND_RIGHT, FF_SLOPE_RIGHT, KICK_PWM_RIGHT};
    t.kickMs = KICK_MS;
    t.kp = K_P;
    t.ki = K_I;
    t.trimMaxPwm = TRIM_MAX_PWM;
    t.countsPerTurn = ENCODER_CPR;
    t.closedLoop = CLOSED_LOOP != 0;
    return t;
}

const char* encoderFaultName(EncoderFault fault) {
    switch (fault) {
        case EncoderFault::None: return "none";
        case EncoderFault::NoCounts: return "no_counts";
        case EncoderFault::WrongDirection: return "wrong_dir";
        case EncoderFault::Implausible: return "implausible";
        case EncoderFault::TrimSaturated: return "trim_sat";
    }
    return "?";
}

WheelController::WheelController(const WheelTuning& wheel, const ControlTuning& common)
    : m_wheel(wheel), m_common(common),
      m_pid(common.kp, common.ki, 0, -common.trimMaxPwm, common.trimMaxPwm),
      m_sinceKickMs(KICK_MIN_INTERVAL_MS) {}

bool WheelController::feedbackActive() const {
    return m_common.closedLoop && m_moving && m_fault == EncoderFault::None && m_proven &&
           m_motionMs >= VELOCITY_WINDOW_TICKS * CONTROL_PERIOD_MS &&
           m_sinceKickMs >= m_common.kickMs + VELOCITY_WINDOW_TICKS * CONTROL_PERIOD_MS;
}

float WheelController::update(float targetMps, int32_t count) {
    trackCount(count);
    if (fabsf(targetMps) < STOP_SPEED_MPS) {
        stop();
        return 0;
    }
    m_target = targetMps;

    const int dir = targetMps > 0 ? 1 : -1;
    if (!m_moving || dir != m_dir) {
        startMotion(dir, count);
    } else {
        m_motionMs += CONTROL_PERIOD_MS;
    }
    checkEncoder(count);
    if (m_kickWanted && m_sinceKickMs >= KICK_MIN_INTERVAL_MS) {
        m_kickWanted = false;
        m_kickLeftMs = (uint32_t)m_common.kickMs;
        m_sinceKickMs = 0;
    }

    m_feedForward = dir * (m_wheel.deadbandPwm + m_wheel.slopePwmPerMps * fabsf(targetMps));

    if (!m_common.closedLoop || m_fault != EncoderFault::None) {
        m_trim = 0;
        m_pid.reset();
    } else if (feedbackActive()) {
        m_pid.setTunings(m_common.kp, m_common.ki, 0);
        m_pid.setOutputLimits(-m_common.trimMaxPwm, m_common.trimMaxPwm);
        m_trim = m_pid.compute(targetMps, m_measured, DT_SECONDS);
    }  // else: the encoder is being checked again after a stall, so hold the trim where it was

    float out = m_feedForward + m_trim;
    if (out * dir < 0) {
        out = 0;  // the trim can slow a wheel but must not drive it backwards
    }
    if (m_kickLeftMs > 0) {
        out = dir * fmaxf(fabsf(out), m_wheel.kickPwm);
        m_kickLeftMs = m_kickLeftMs > CONTROL_PERIOD_MS ? m_kickLeftMs - CONTROL_PERIOD_MS : 0;
    }
    if (out > PWM_MAX) out = PWM_MAX;
    if (out < -PWM_MAX) out = -PWM_MAX;
    return out;
}

void WheelController::idle(int32_t count) {
    trackCount(count);
    stop();
}

void WheelController::trackCount(int32_t count) {
    if (!m_historyReady) {
        for (int32_t& h : m_history) h = count;
        m_historyReady = true;
    }
    m_windowCounts = count - m_history[m_historyHead];
    m_history[m_historyHead] = count;
    m_historyHead = (m_historyHead + 1) % VELOCITY_WINDOW_TICKS;

    const float metersPerCount = 2.0f * PI_F * WHEEL_RADIUS / m_common.countsPerTurn;
    m_measured = m_windowCounts * metersPerCount / (VELOCITY_WINDOW_TICKS * DT_SECONDS);

    if (m_sinceKickMs < 60000) m_sinceKickMs += CONTROL_PERIOD_MS;
}

void WheelController::startMotion(int dir, int32_t count) {
    m_moving = true;
    m_dir = dir;
    m_motionMs = 0;
    m_fault = EncoderFault::None;
    m_trim = 0;
    m_pid.reset();
    m_saturatedMs = 0;
    m_settleMs = dir * m_windowCounts < 0 ? ENC_REVERSAL_SETTLE_MS : ENC_WRONG_DIR_SETTLE_MS;
    requireProof(count);
}

// The trim is only used while the encoder shows the wheel moving the requested way. Starting with
// a kick means a working encoder has something to show, even if the feed-forward is too low.
void WheelController::requireProof(int32_t count) {
    m_proven = false;
    m_proofStart = count;
    m_proofMs = 0;
    m_stillMs = 0;
    m_kickWanted = m_dir * m_windowCounts < ENC_PROOF_COUNTS;  // not if it's already turning that way
}

void WheelController::checkEncoder(int32_t count) {
    if (m_fault != EncoderFault::None) {
        return;
    }
    if (fabsf(m_measured) > ENC_MAX_MPS) {
        raiseFault(EncoderFault::Implausible);
        return;
    }
    if (m_motionMs >= m_settleMs && m_dir * m_windowCounts <= -ENC_WRONG_DIR_COUNTS) {
        raiseFault(EncoderFault::WrongDirection);
        return;
    }
    if (!m_proven) {
        if (m_dir * (count - m_proofStart) < 0) {
            m_proofStart = count;  // still rolling the old way after a reversal: the time doesn't run
        } else if (!m_kickWanted) {
            m_proofMs += CONTROL_PERIOD_MS;
        }
        if (m_dir * (count - m_proofStart) >= ENC_PROOF_COUNTS) {
            m_proven = true;
            m_stillMs = 0;
            m_kickWanted = false;
        } else if (m_proofMs >= ENC_PROOF_TIMEOUT_MS) {
            raiseFault(EncoderFault::NoCounts);
        }
        return;
    }
    m_stillMs = abs(m_windowCounts) < ENC_STILL_COUNTS ? m_stillMs + CONTROL_PERIOD_MS : 0;
    if (m_stillMs >= ENC_STILL_MS) {
        requireProof(count);  // a stall if the kick moves it, a dead encoder if not
        return;
    }
    // Only a trim adding power is a fault. One at its limit taking power away is doing the safe
    // thing, and dropping it would speed the wheel up.
    m_saturatedMs = m_dir * m_trim >= m_common.trimMaxPwm - 0.5f ? m_saturatedMs + CONTROL_PERIOD_MS : 0;
    if (m_saturatedMs >= TRIM_SATURATED_MS) {
        raiseFault(EncoderFault::TrimSaturated);
    }
}

void WheelController::raiseFault(EncoderFault fault) {
    m_fault = fault;
    m_faultCount++;
    m_proven = false;
    m_kickWanted = false;
}

void WheelController::stop() {
    m_moving = false;
    m_proven = false;
    m_kickWanted = false;
    m_kickLeftMs = 0;
    m_target = 0;
    m_feedForward = 0;
    m_trim = 0;
    m_pid.reset();
}

void DriveCommand::setVelocity(float leftMps, float rightMps, uint32_t nowMs) {
    left = leftMps;
    right = rightMps;
    velocityMs = nowMs;
    hasVelocity = true;
    hasForce = false;
}

void DriveCommand::setForce(int leftPwm, int rightPwm, uint32_t nowMs) {
    forceLeft = leftPwm;
    forceRight = rightPwm;
    forceUntilMs = nowMs + FORCE_TEST_MS;
    hasForce = leftPwm != 0 || rightPwm != 0;
    hasVelocity = false;
}

bool DriveCommand::velocityFresh(uint32_t nowMs) const {
    return hasVelocity && (int32_t)(nowMs - velocityMs) < CMD_TIMEOUT;
}

bool DriveCommand::forceActive(uint32_t nowMs) const {
    return hasForce && (int32_t)(nowMs - forceUntilMs) < 0;
}

void DriveCommand::expire(uint32_t nowMs) {
    hasVelocity = velocityFresh(nowMs);
    hasForce = forceActive(nowMs);
}

DriveController::DriveController(const ControlTuning& tuning)
    : m_left(tuning.left, tuning), m_right(tuning.right, tuning) {}

void DriveController::update(const DriveCommand& cmd, uint32_t nowMs, int32_t leftCount, int32_t rightCount,
                             float& leftPwm, float& rightPwm) {
    if (cmd.forceActive(nowMs)) {
        m_left.idle(leftCount);
        m_right.idle(rightCount);
        leftPwm = cmd.forceLeft;
        rightPwm = cmd.forceRight;
        return;
    }
    const bool fresh = cmd.velocityFresh(nowMs);
    leftPwm = m_left.update(fresh ? cmd.left : 0, leftCount);
    rightPwm = m_right.update(fresh ? cmd.right : 0, rightCount);
}

int32_t counterDelta(int16_t now, int16_t last, int16_t limit) {
    int32_t delta = (int32_t)now - last;
    if (delta > limit / 2) delta -= limit;
    else if (delta < -limit / 2) delta += limit;
    return delta;
}
