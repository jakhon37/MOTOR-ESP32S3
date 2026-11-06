// MOTOR-ESP32S3/lib/SpeedControl/SpeedControl.h
// Wheel speed control: feed-forward from the request, with a closed-loop trim on top that is only
// used while the encoder proves it works. No Arduino dependency, so it can be tested on a PC.
#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdint.h>
#include "PIDController.h"
#include "config.h"

struct WheelTuning {
    float deadbandPwm;
    float slopePwmPerMps;
    float kickPwm;
};

struct ControlTuning {
    WheelTuning left;
    WheelTuning right;
    float kickMs;
    float kp;
    float ki;
    float trimMaxPwm;
    float countsPerTurn;
    bool closedLoop;
};

ControlTuning defaultTuning();  // from config.h

enum class EncoderFault : uint8_t {
    None,
    NoCounts,        // driven, but the count did not move the right way
    WrongDirection,  // the count moves against the request: LEFT/RIGHT_ENCODER_SIGN or the motor wires
    Implausible,     // faster than the wheel can turn: noise on the encoder lines
    TrimSaturated,   // the trim stayed at its limit: bad feed-forward, a blocked wheel or lost counts
};

const char* encoderFaultName(EncoderFault fault);

class WheelController {
public:
    WheelController(const WheelTuning& wheel, const ControlTuning& common);

    // One control tick. count is the wheel's running total from the encoder, with the sign
    // that goes up on a positive request. Returns signed PWM.
    float update(float targetMps, int32_t count);

    // One control tick with the output not under this controller's control.
    void idle(int32_t count);

    float targetMps() const { return m_target; }
    float measuredMps() const { return m_measured; }
    float feedForwardPwm() const { return m_feedForward; }
    float trimPwm() const { return m_trim; }
    float outputPwm() const { return m_output; }
    bool feedbackActive() const;
    EncoderFault fault() const { return m_fault; }  // kept after a stop, until the next start
    uint32_t faultCount() const { return m_faultCount; }

private:
    void trackCount(int32_t count);
    void startMotion(int dir, int32_t count);
    void requireProof(int32_t count);
    void checkEncoder(int32_t count);
    void raiseFault(EncoderFault fault);
    void stop();

    const WheelTuning& m_wheel;
    const ControlTuning& m_common;
    PIDController m_pid;

    int32_t m_history[VELOCITY_WINDOW_TICKS];
    int m_historyHead = 0;
    bool m_historyReady = false;
    int32_t m_windowCounts = 0;

    float m_target = 0;
    float m_measured = 0;
    float m_feedForward = 0;
    float m_trim = 0;
    float m_output = 0;

    bool m_moving = false;
    int m_dir = 0;
    uint32_t m_motionMs = 0;
    uint32_t m_sinceKickMs = 0;
    uint32_t m_kickLeftMs = 0;

    bool m_proven = false;
    int32_t m_proofStart = 0;
    uint32_t m_proofMs = 0;
    uint32_t m_stillMs = 0;
    uint32_t m_saturatedMs = 0;
    EncoderFault m_fault = EncoderFault::None;
    uint32_t m_faultCount = 0;
};

// What the serial side asks for. Written by the serial loop and read by the control tick.
struct DriveCommand {
    float left = 0;
    float right = 0;
    uint32_t velocityMs = 0;
    bool hasVelocity = false;

    int forceLeft = 0;
    int forceRight = 0;
    uint32_t forceUntilMs = 0;
    bool hasForce = false;

    void setVelocity(float leftMps, float rightMps, uint32_t nowMs);  // ends a force test
    void setForce(int leftPwm, int rightPwm, uint32_t nowMs);         // open-loop test, zero ends it
};

class DriveController {
public:
    explicit DriveController(const ControlTuning& tuning);

    void update(const DriveCommand& cmd, uint32_t nowMs, int32_t leftCount, int32_t rightCount,
                float& leftPwm, float& rightPwm);

    const WheelController& left() const { return m_left; }
    const WheelController& right() const { return m_right; }

private:
    WheelController m_left;
    WheelController m_right;
};

// Change in a hardware counter that resets to 0 at +limit and at -limit, given the last reading.
// Correct as long as it moved less than limit / 2 between readings.
int32_t counterDelta(int16_t now, int16_t last, int16_t limit);

#endif
