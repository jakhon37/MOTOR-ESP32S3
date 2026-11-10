// MOTOR-ESP32S3/test/test_control/test_main.cpp
// Runs on the PC with no board: pio test -e native
// The wheel below is a model, not a measurement of the robot. These tests show the control logic
// does what it should across a range of motors and encoder faults, not what the real wheels do.
#include <math.h>
#include <unity.h>
#include <initializer_list>
#include "CommandParser.h"
#include "SpeedControl.h"

static const float DT = CONTROL_PERIOD_MS / 1000.0f;
static const float TWO_PI_R = 2.0f * 3.14159265f * WHEEL_RADIUS;
static const float COUNTS_PER_TURN = 1497;  // the robot's encoders, but fixed here like the tuning below

// A geared motor: first-order speed response, a deadband while turning, and static friction that
// takes more PWM to break than the deadband.
struct Wheel {
    float deadband = 200;
    float slope = 1100;
    float staticPwm = 330;
    float tau = 0.05f;
    bool blocked = false;

    float speed = 0;
    double counts = 0;

    void step(float pwm) {
        const float mag = fabsf(pwm);
        if (blocked || (speed == 0 && mag < staticPwm)) {
            speed = 0;
        } else {
            const float steady = mag > deadband ? copysignf((mag - deadband) / slope, pwm) : 0;
            speed += (steady - speed) * DT / tau;
            if (steady == 0 && fabsf(speed) < 0.002f) speed = 0;
        }
        counts += speed * DT * COUNTS_PER_TURN / TWO_PI_R;
    }
    int32_t count() const { return (int32_t)floor(counts); }
};

// The tests' own tuning, made for the default wheel above: its feed-forward is that wheel's deadband
// and slope, and its kick is over the wheel's static friction. config.h holds what was measured on
// the robot, which isn't this wheel, so the tests don't use it.
static ControlTuning testTuning() {
    const Wheel wheel;
    ControlTuning t;
    t.left = {wheel.deadband, wheel.slope, 550};
    t.right = t.left;
    t.kickMs = 40;
    t.kp = 300;
    t.ki = 2000;
    t.trimMaxPwm = 250;
    t.countsPerTurn = COUNTS_PER_TURN;
    t.closedLoop = true;
    return t;
}

struct Rig {
    ControlTuning tuning = testTuning();
    WheelController controller{tuning.left, tuning};
    Wheel wheel;
    float lastPwm = 0;
    float maxPwm = 0;
    float maxTrim = 0;

    // The output on feed-forward alone, with no trim or kick.
    float feedForward(float target) const {
        return tuning.left.deadbandPwm + tuning.left.slopePwmPerMps * target;
    }

    void run(float target, float seconds) { runWith(target, seconds, [](int32_t c) { return c; }); }

    // encoder maps the true count to what the firmware sees.
    template <typename Encoder>
    void runWith(float target, float seconds, Encoder encoder) {
        const int ticks = (int)lroundf(seconds / DT);
        for (int i = 0; i < ticks; i++) {
            lastPwm = controller.update(target, encoder(wheel.count()));
            maxPwm = fmaxf(maxPwm, fabsf(lastPwm));
            maxTrim = fmaxf(maxTrim, fabsf(controller.trimPwm()));
            wheel.step(lastPwm);
        }
    }

    // True mean speed over the given time, with the wheel at target.
    float meanSpeed(float target, float seconds) {
        const double start = wheel.counts;
        run(target, seconds);
        return (float)((wheel.counts - start) * TWO_PI_R / COUNTS_PER_TURN / seconds);
    }

    // Same, through an encoder that shows nothing.
    float meanSpeedDeadEncoder(float target, float seconds) {
        const double start = wheel.counts;
        runWith(target, seconds, [](int32_t) { return 0; });
        return (float)((wheel.counts - start) * TWO_PI_R / COUNTS_PER_TURN / seconds);
    }
};

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------------------------
// Requests and the stop rule

void test_request_under_stop_speed_gives_zero_pwm() {
    Rig rig;
    for (float v : {0.0f, 0.005f, -0.005f, 0.0099f, -0.0099f}) {
        TEST_ASSERT_EQUAL_FLOAT(0, rig.controller.update(v, 0));
    }
    rig.run(0.2f, 1.0f);
    TEST_ASSERT_GREATER_THAN_FLOAT(0, rig.lastPwm);
    rig.run(0.0f, 0.02f);
    TEST_ASSERT_EQUAL_FLOAT(0, rig.lastPwm);
}

void test_start_kicks_then_settles_to_feed_forward() {
    Rig rig;
    rig.tuning.closedLoop = false;
    TEST_ASSERT_EQUAL_FLOAT(rig.tuning.left.kickPwm, rig.controller.update(0.02f, 0));
    rig.run(0.02f, 0.2f);
    TEST_ASSERT_EQUAL_FLOAT(rig.feedForward(0.02f), rig.lastPwm);
}

void test_negative_request_mirrors_positive() {
    Rig fwd, rev;
    fwd.run(0.15f, 2.0f);
    rev.run(-0.15f, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, fwd.lastPwm, -rev.lastPwm);
    TEST_ASSERT_FLOAT_WITHIN(0.003f, fwd.wheel.speed, -rev.wheel.speed);
}

// ---------------------------------------------------------------------------------------------
// Speed against request

static const float REQUESTS[] = {0.02f, 0.03f, 0.05f, 0.08f, 0.10f, 0.15f, 0.20f, 0.25f};

void test_closed_loop_tracks_with_correct_feed_forward() {
    for (float v : REQUESTS) {
        Rig rig;
        rig.run(v, 2.0f);
        TEST_ASSERT_FLOAT_WITHIN(0.1f * v + 0.002f, v, rig.meanSpeed(v, 2.0f));
        TEST_ASSERT_EQUAL(EncoderFault::None, rig.controller.fault());
    }
}

// The motor isn't the one the tuning was made for. The trim must make up the difference.
void test_closed_loop_tracks_with_wrong_feed_forward() {
    const struct { float deadband, slope, staticPwm, tau; } motors[] = {
        {100, 700, 200, 0.03f},
        {120, 1100, 220, 0.05f},
        {280, 1100, 400, 0.05f},
        {280, 1500, 400, 0.10f},
        {200, 800, 330, 0.02f},
    };
    for (const auto& m : motors) {
        for (float v : REQUESTS) {
            Rig rig;
            rig.wheel.deadband = m.deadband;
            rig.wheel.slope = m.slope;
            rig.wheel.staticPwm = m.staticPwm;
            rig.wheel.tau = m.tau;
            rig.run(v, 4.0f);
            TEST_ASSERT_FLOAT_WITHIN(0.15f * v + 0.003f, v, rig.meanSpeed(v, 2.0f));
            TEST_ASSERT_EQUAL(EncoderFault::None, rig.controller.fault());
        }
    }
}

void test_different_requests_give_different_speeds_closed_loop() {
    float last = 0;
    for (float v : REQUESTS) {
        Rig rig;
        rig.run(v, 2.0f);
        const float speed = rig.meanSpeed(v, 2.0f);
        TEST_ASSERT_GREATER_THAN_FLOAT(last, speed);
        last = speed;
    }
}

// With no encoders, speed still follows the request, as well as the feed-forward fits the motor.
void test_dead_encoder_still_proportional_on_feed_forward() {
    float last = 0;
    for (float v : REQUESTS) {
        Rig rig;
        rig.runWith(v, 2.0f, [](int32_t) { return 0; });
        const float speed = rig.meanSpeedDeadEncoder(v, 2.0f);
        TEST_ASSERT_FLOAT_WITHIN(0.1f * v + 0.002f, v, speed);
        TEST_ASSERT_GREATER_THAN_FLOAT(last, speed);
        last = speed;
        TEST_ASSERT_EQUAL(EncoderFault::NoCounts, rig.controller.fault());
    }
}

// ---------------------------------------------------------------------------------------------
// Encoder faults must never push the wheel harder than feed-forward for long

void test_dead_encoder_never_trims() {
    Rig rig;
    rig.runWith(0.02f, 5.0f, [](int32_t) { return 0; });
    TEST_ASSERT_EQUAL(EncoderFault::NoCounts, rig.controller.fault());
    TEST_ASSERT_EQUAL_FLOAT(0, rig.maxTrim);
    TEST_ASSERT_EQUAL_FLOAT(rig.feedForward(0.02f), rig.lastPwm);
    TEST_ASSERT_EQUAL_FLOAT(rig.tuning.left.kickPwm, rig.maxPwm);
}

// With B dead, the hardware decoder steps back and forth on every A edge.
void test_dead_b_channel_counts_as_no_counts() {
    Rig rig;
    rig.runWith(0.1f, 2.0f, [](int32_t c) { return -(c & 1); });
    TEST_ASSERT_EQUAL(EncoderFault::NoCounts, rig.controller.fault());
    TEST_ASSERT_EQUAL_FLOAT(0, rig.maxTrim);
}

void test_reversed_encoder_is_caught_without_runaway() {
    Rig rig;
    rig.runWith(0.1f, 1.0f, [](int32_t c) { return -c; });
    TEST_ASSERT_EQUAL(EncoderFault::WrongDirection, rig.controller.fault());
    rig.runWith(0.1f, 2.0f, [](int32_t c) { return -c; });
    TEST_ASSERT_EQUAL_FLOAT(0, rig.maxTrim);
    TEST_ASSERT_EQUAL_FLOAT(rig.feedForward(0.1f), rig.lastPwm);
    TEST_ASSERT_EQUAL_FLOAT(rig.tuning.left.kickPwm, rig.maxPwm);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.1f, rig.wheel.speed);
}

void test_encoder_dying_while_driving_falls_back_to_feed_forward() {
    Rig rig;
    rig.run(0.1f, 2.0f);
    const int32_t frozen = rig.wheel.count();
    rig.runWith(0.1f, 2.0f, [frozen](int32_t) { return frozen; });
    TEST_ASSERT_EQUAL(EncoderFault::NoCounts, rig.controller.fault());
    TEST_ASSERT_EQUAL_FLOAT(0, rig.controller.trimPwm());
    TEST_ASSERT_EQUAL_FLOAT(rig.feedForward(0.1f), rig.lastPwm);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.1f, rig.wheel.speed);
}

void test_losing_most_counts_ends_in_trim_saturated() {
    Rig rig;
    rig.runWith(0.1f, 6.0f, [](int32_t c) { return c / 5; });
    TEST_ASSERT_EQUAL(EncoderFault::TrimSaturated, rig.controller.fault());
    TEST_ASSERT_EQUAL_FLOAT(rig.feedForward(0.1f), rig.lastPwm);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.1f, rig.wheel.speed);
}

// The feed-forward is far too high for this motor, so the trim sits at its limit taking power away.
// Dropping it would make the wheel run away from the request.
void test_trim_at_its_limit_taking_power_away_is_kept() {
    Rig rig;
    rig.wheel.deadband = 100;
    rig.wheel.slope = 400;
    rig.wheel.staticPwm = 200;
    rig.run(0.25f, 6.0f);
    TEST_ASSERT_EQUAL(EncoderFault::None, rig.controller.fault());
    TEST_ASSERT_EQUAL_FLOAT(-rig.tuning.trimMaxPwm, rig.controller.trimPwm());
    TEST_ASSERT_LESS_THAN_FLOAT(0.35f, rig.wheel.speed);
}

void test_noise_burst_is_implausible() {
    Rig rig;
    rig.run(0.1f, 2.0f);
    rig.runWith(0.1f, 0.5f, [](int32_t c) { return c + 100000; });
    TEST_ASSERT_EQUAL(EncoderFault::Implausible, rig.controller.fault());
    TEST_ASSERT_EQUAL_FLOAT(rig.feedForward(0.1f), rig.lastPwm);
}

void test_blocked_wheel_does_not_wind_up() {
    Rig rig;
    rig.wheel.blocked = true;
    rig.run(0.05f, 5.0f);
    TEST_ASSERT_EQUAL(EncoderFault::NoCounts, rig.controller.fault());
    TEST_ASSERT_EQUAL_FLOAT(rig.feedForward(0.05f), rig.lastPwm);
    TEST_ASSERT_EQUAL_FLOAT(rig.tuning.left.kickPwm, rig.maxPwm);
}

void test_fault_clears_at_the_next_start_and_is_counted() {
    Rig rig;
    rig.wheel.blocked = true;
    rig.run(0.05f, 1.0f);
    rig.run(0, 0.5f);
    TEST_ASSERT_EQUAL(EncoderFault::NoCounts, rig.controller.fault());
    rig.wheel.blocked = false;
    rig.run(0.05f, 2.0f);
    TEST_ASSERT_EQUAL(EncoderFault::None, rig.controller.fault());
    TEST_ASSERT_EQUAL_UINT32(1, rig.controller.faultCount());
    TEST_ASSERT_TRUE(rig.controller.feedbackActive());
}

// ---------------------------------------------------------------------------------------------
// Stalls, reversals and flicker

// The feed-forward is too low to keep this wheel turning. The first kick shows the encoder works,
// so the trim is allowed to build until the wheel keeps going.
void test_stall_at_low_speed_recovers_through_the_trim() {
    Rig rig;
    rig.wheel.deadband = 300;
    rig.wheel.staticPwm = 420;
    rig.run(0.03f, 10.0f);
    TEST_ASSERT_EQUAL(EncoderFault::None, rig.controller.fault());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.03f, rig.meanSpeed(0.03f, 2.0f));
}

void test_reversal_tracks_without_a_fault() {
    Rig rig;
    rig.run(0.2f, 2.0f);
    rig.run(-0.2f, 3.0f);
    TEST_ASSERT_EQUAL(EncoderFault::None, rig.controller.fault());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -0.2f, rig.meanSpeed(-0.2f, 1.0f));
}

void test_reversal_into_a_low_speed_is_not_a_fault() {
    Rig rig;
    rig.run(-0.25f, 2.0f);
    rig.run(0.02f, 3.0f);
    TEST_ASSERT_EQUAL(EncoderFault::None, rig.controller.fault());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.02f, rig.meanSpeed(0.02f, 2.0f));
}

// A heavier wheel takes longer to come round. That's not a dead or reversed encoder.
void test_slow_reversal_is_not_a_fault() {
    for (float tau : {0.10f, 0.15f, 0.20f}) {
        for (float request : {0.02f, 0.10f, 0.25f}) {
            Rig rig;
            rig.wheel.tau = tau;
            rig.run(-0.25f, 2.0f);
            rig.run(request, 3.0f);
            TEST_ASSERT_EQUAL(EncoderFault::None, rig.controller.fault());
            TEST_ASSERT_FLOAT_WITHIN(0.1f * request + 0.005f, request, rig.meanSpeed(request, 2.0f));
        }
    }
}

// A restart too soon after the last kick must wait for its kick, not go without one: feed-forward
// alone can't start this wheel, and it would be stuck until the next stop.
void test_restart_soon_after_a_kick_still_gets_one() {
    Rig rig;
    rig.run(0.012f, 0.03f);
    rig.run(0, 0.2f);
    rig.wheel.speed = 0;
    rig.run(0.012f, 4.0f);
    TEST_ASSERT_EQUAL(EncoderFault::None, rig.controller.fault());
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.012f, rig.meanSpeed(0.012f, 2.0f));
}

void test_restart_while_still_rolling_gets_no_kick() {
    Rig rig;
    rig.run(0.2f, 2.0f);
    rig.run(0, 0.02f);
    rig.maxPwm = 0;
    rig.run(0.05f, 0.3f);
    TEST_ASSERT_LESS_THAN_FLOAT(rig.tuning.left.kickPwm, rig.maxPwm);
}

void test_request_flickering_around_stop_speed_limits_kicks() {
    Rig rig;
    const float kick = rig.tuning.left.kickPwm;
    int kicks = 0;
    float last = 0;
    const int ticks = 200;  // 2 s
    for (int i = 0; i < ticks; i++) {
        const float request = (i / 5) % 2 == 0 ? 0.012f : 0.008f;  // flips every 50 ms
        const float pwm = rig.controller.update(request, rig.wheel.count());
        if (pwm >= kick && last < kick) kicks++;
        last = pwm;
        rig.wheel.step(pwm);
    }
    TEST_ASSERT_LESS_OR_EQUAL_INT(2000 / KICK_MIN_INTERVAL_MS + 1, kicks);
}

// ---------------------------------------------------------------------------------------------
// Timeout and the force test

void test_no_drive_before_the_first_command() {
    ControlTuning tuning = testTuning();
    DriveController drive(tuning);
    DriveCommand cmd;
    float l = 1, r = 1;
    drive.update(cmd, 10, 0, 0, l, r);
    TEST_ASSERT_EQUAL_FLOAT(0, l);
    TEST_ASSERT_EQUAL_FLOAT(0, r);
}

static void assertTimeoutAt(uint32_t start) {
    ControlTuning tuning = testTuning();
    DriveController drive(tuning);
    DriveCommand cmd;
    cmd.setVelocity(0.2f, -0.2f, start);
    float l, r;
    drive.update(cmd, start + CMD_TIMEOUT - 1, 0, 0, l, r);
    TEST_ASSERT_GREATER_THAN_FLOAT(0, l);
    TEST_ASSERT_LESS_THAN_FLOAT(0, r);
    drive.update(cmd, start + CMD_TIMEOUT, 0, 0, l, r);
    TEST_ASSERT_EQUAL_FLOAT(0, l);
    TEST_ASSERT_EQUAL_FLOAT(0, r);

    cmd.setVelocity(0.2f, -0.2f, start + CMD_TIMEOUT + 100);
    drive.update(cmd, start + CMD_TIMEOUT + 110, 0, 0, l, r);
    TEST_ASSERT_GREATER_THAN_FLOAT(0, l);
}

void test_command_timeout_stops_and_a_new_command_restarts() {
    assertTimeoutAt(1000);
    assertTimeoutAt(0xFFFFFF00u);  // across millis() wrapping round
}

void test_force_runs_raw_pwm_then_stops_by_itself() {
    ControlTuning tuning = testTuning();
    DriveController drive(tuning);
    DriveCommand cmd;
    cmd.setForce(300, -700, 1000);
    float l, r;
    drive.update(cmd, 1000 + FORCE_TEST_MS - 1, 0, 0, l, r);
    TEST_ASSERT_EQUAL_FLOAT(300, l);
    TEST_ASSERT_EQUAL_FLOAT(-700, r);
    drive.update(cmd, 1000 + FORCE_TEST_MS, 0, 0, l, r);
    TEST_ASSERT_EQUAL_FLOAT(0, l);
    TEST_ASSERT_EQUAL_FLOAT(0, r);
}

void test_force_and_velocity_cancel_each_other() {
    ControlTuning tuning = testTuning();
    DriveController drive(tuning);
    DriveCommand cmd;
    float l, r;

    cmd.setForce(300, 300, 1000);
    cmd.setVelocity(0, 0, 1010);
    drive.update(cmd, 1020, 0, 0, l, r);
    TEST_ASSERT_EQUAL_FLOAT(0, l);

    cmd.setVelocity(0.2f, 0.2f, 1100);
    cmd.setForce(0, 0, 1110);
    drive.update(cmd, 1120, 0, 0, l, r);
    TEST_ASSERT_EQUAL_FLOAT(0, l);
    TEST_ASSERT_EQUAL_FLOAT(0, r);
}

// In the firmware, expire() runs every tick. Without it, an old command would count as new again
// when millis() comes round, days later.
void test_expired_commands_never_come_back() {
    ControlTuning tuning = testTuning();
    DriveController drive(tuning);
    float l, r;

    DriveCommand velocity;
    velocity.setVelocity(0.2f, 0.2f, 1000);
    velocity.expire(1000 + CMD_TIMEOUT - 1);
    TEST_ASSERT_TRUE(velocity.hasVelocity);
    velocity.expire(1000 + CMD_TIMEOUT);
    TEST_ASSERT_FALSE(velocity.hasVelocity);
    for (uint32_t later : {0x80000000u, 0xFFFFFFFFu, 1100u}) {
        drive.update(velocity, later, 0, 0, l, r);
        TEST_ASSERT_EQUAL_FLOAT(0, l);
    }

    DriveCommand force;
    force.setForce(300, 300, 1000);
    force.expire(1000 + FORCE_TEST_MS - 1);
    TEST_ASSERT_TRUE(force.hasForce);
    force.expire(1000 + FORCE_TEST_MS);
    TEST_ASSERT_FALSE(force.hasForce);
    for (uint32_t later : {0x80000000u, 0xFFFFFFFFu, 1100u}) {
        drive.update(force, later, 0, 0, l, r);
        TEST_ASSERT_EQUAL_FLOAT(0, l);
    }
}

// ---------------------------------------------------------------------------------------------
// Counter wrap-around

void test_counter_delta_across_the_limits() {
    TEST_ASSERT_EQUAL_INT32(20, counterDelta(120, 100, 30000));
    TEST_ASSERT_EQUAL_INT32(-20, counterDelta(80, 100, 30000));
    TEST_ASSERT_EQUAL_INT32(20, counterDelta(10, 29990, 30000));
    TEST_ASSERT_EQUAL_INT32(-20, counterDelta(-10, -29990, 30000));
}

// ---------------------------------------------------------------------------------------------
// Serial protocol

void test_parses_what_the_bridge_sends() {
    const Command c = parseCommand("m 0.100 -0.100");
    TEST_ASSERT_EQUAL(CommandType::Velocity, c.type);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, c.a);
    TEST_ASSERT_EQUAL_FLOAT(-0.1f, c.b);
    TEST_ASSERT_EQUAL(CommandType::Velocity, parseCommand("m -0.000 -0.000").type);
    TEST_ASSERT_EQUAL(CommandType::Velocity, parseCommand("  m\t0.1  0.2  ").type);
}

void test_rejects_damaged_velocity_lines() {
    for (const char* line : {"", "m", "m 0.1", "m x y", "m 0.1 y", "m nan 0", "m 0 inf", "m 0.1 0.1m 0.2 0.2",
                             "m 0.1 0.2 0.3", "0.1 0.2", "x 0.1 0.2"}) {
        TEST_ASSERT_EQUAL_MESSAGE(CommandType::Invalid, parseCommand(line).type, line);
    }
}

void test_parses_test_commands() {
    Command c = parseCommand("f 300");
    TEST_ASSERT_EQUAL(CommandType::ForcePwm, c.type);
    TEST_ASSERT_EQUAL_FLOAT(300, c.a);
    TEST_ASSERT_EQUAL_FLOAT(300, c.b);
    c = parseCommand("f 300 -200");
    TEST_ASSERT_EQUAL_FLOAT(-200, c.b);
    TEST_ASSERT_EQUAL(CommandType::Invalid, parseCommand("f").type);
    TEST_ASSERT_EQUAL(CommandType::Diagnostics, parseCommand("d").type);
    TEST_ASSERT_EQUAL(CommandType::GetTuning, parseCommand("g").type);
    c = parseCommand("g dead_l 245.5");
    TEST_ASSERT_EQUAL(CommandType::SetTuning, c.type);
    TEST_ASSERT_EQUAL_STRING("dead_l", c.name);
    TEST_ASSERT_EQUAL_FLOAT(245.5f, c.a);
    TEST_ASSERT_EQUAL(CommandType::Invalid, parseCommand("g dead_l").type);
    TEST_ASSERT_EQUAL(CommandType::Invalid, parseCommand("g a_name_that_is_too_long 1").type);
}

// ---------------------------------------------------------------------------------------------
// config.h's tuning

// The tests above run on their own tuning. The one measured on the robot is only checked for what
// any tuning needs: each wheel gets its own values, and they stay within the PWM range.
void test_config_tuning_is_within_the_limits() {
    const ControlTuning t = defaultTuning();
    TEST_ASSERT_EQUAL_FLOAT(FF_DEADBAND_LEFT, t.left.deadbandPwm);
    TEST_ASSERT_EQUAL_FLOAT(FF_SLOPE_LEFT, t.left.slopePwmPerMps);
    TEST_ASSERT_EQUAL_FLOAT(KICK_PWM_LEFT, t.left.kickPwm);
    TEST_ASSERT_EQUAL_FLOAT(FF_DEADBAND_RIGHT, t.right.deadbandPwm);
    TEST_ASSERT_EQUAL_FLOAT(FF_SLOPE_RIGHT, t.right.slopePwmPerMps);
    TEST_ASSERT_EQUAL_FLOAT(KICK_PWM_RIGHT, t.right.kickPwm);
    for (const WheelTuning& w : {t.left, t.right}) {
        TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(0, w.deadbandPwm);
        TEST_ASSERT_GREATER_THAN_FLOAT(0, w.slopePwmPerMps);
        TEST_ASSERT_GREATER_THAN_FLOAT(w.deadbandPwm, w.kickPwm);  // starting takes more than keeping going
        TEST_ASSERT_LESS_OR_EQUAL_FLOAT(PWM_MAX, w.kickPwm);
        // Feed-forward at 0.25 m/s, the top of the range FIRST_DRIVE.md checks.
        TEST_ASSERT_LESS_OR_EQUAL_FLOAT(PWM_MAX, w.deadbandPwm + w.slopePwmPerMps * 0.25f);
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(0, t.trimMaxPwm);
    TEST_ASSERT_LESS_THAN_FLOAT(PWM_MAX, t.trimMaxPwm);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_request_under_stop_speed_gives_zero_pwm);
    RUN_TEST(test_start_kicks_then_settles_to_feed_forward);
    RUN_TEST(test_negative_request_mirrors_positive);
    RUN_TEST(test_closed_loop_tracks_with_correct_feed_forward);
    RUN_TEST(test_closed_loop_tracks_with_wrong_feed_forward);
    RUN_TEST(test_different_requests_give_different_speeds_closed_loop);
    RUN_TEST(test_dead_encoder_still_proportional_on_feed_forward);
    RUN_TEST(test_dead_encoder_never_trims);
    RUN_TEST(test_dead_b_channel_counts_as_no_counts);
    RUN_TEST(test_reversed_encoder_is_caught_without_runaway);
    RUN_TEST(test_encoder_dying_while_driving_falls_back_to_feed_forward);
    RUN_TEST(test_losing_most_counts_ends_in_trim_saturated);
    RUN_TEST(test_trim_at_its_limit_taking_power_away_is_kept);
    RUN_TEST(test_noise_burst_is_implausible);
    RUN_TEST(test_blocked_wheel_does_not_wind_up);
    RUN_TEST(test_fault_clears_at_the_next_start_and_is_counted);
    RUN_TEST(test_stall_at_low_speed_recovers_through_the_trim);
    RUN_TEST(test_reversal_tracks_without_a_fault);
    RUN_TEST(test_reversal_into_a_low_speed_is_not_a_fault);
    RUN_TEST(test_slow_reversal_is_not_a_fault);
    RUN_TEST(test_restart_soon_after_a_kick_still_gets_one);
    RUN_TEST(test_restart_while_still_rolling_gets_no_kick);
    RUN_TEST(test_request_flickering_around_stop_speed_limits_kicks);
    RUN_TEST(test_no_drive_before_the_first_command);
    RUN_TEST(test_command_timeout_stops_and_a_new_command_restarts);
    RUN_TEST(test_force_runs_raw_pwm_then_stops_by_itself);
    RUN_TEST(test_force_and_velocity_cancel_each_other);
    RUN_TEST(test_expired_commands_never_come_back);
    RUN_TEST(test_counter_delta_across_the_limits);
    RUN_TEST(test_parses_what_the_bridge_sends);
    RUN_TEST(test_rejects_damaged_velocity_lines);
    RUN_TEST(test_parses_test_commands);
    RUN_TEST(test_config_tuning_is_within_the_limits);
    return UNITY_END();
}
