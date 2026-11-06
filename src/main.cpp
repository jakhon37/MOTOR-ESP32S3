// MOTOR-ESP32S3/src/main.cpp
//
// Control task: reads the encoders, runs wheel speed control, sets PWM. Nothing in it blocks, so
// the command timeout still stops the motors if the serial side stalls.
// loop(): serial commands, `e` and `i` telemetry, IMU, LED.
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include "CommandParser.h"
#include "MotorControl.h"
#include "SpeedControl.h"
#include "config.h"

static const uint32_t TELEMETRY_PERIOD_MS = 20;      // 50 Hz, for `e` and for `i`
static const uint32_t IMU_RETRY_MS = 2000;
static const int32_t COMMAND_ACTIVE_MS = 200;        // LED shows blue this long after a command
static const float IMU_MAX_ACCEL = 2 * SENSORS_GRAVITY_STANDARD * 1.01f;  // the +/-2 g range
static const float IMU_MAX_GYRO = 250 * SENSORS_DPS_TO_RADS * 1.01f;      // the +/-250 deg/s range
static const uint8_t IMU_MAX_BAD_READS = 25;         // half a second of them means it's gone

Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;

static ControlTuning tuning = defaultTuning();
static DriveController drive(tuning);

static portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
static DriveCommand sharedCommand;

static volatile float leftPwmNow = 0, rightPwmNow = 0;  // what the motors are getting, for `d`

static bool encodersReady = false;
static bool imuReady = false;
static uint8_t imuBadInARow = 0;
static uint32_t imuBadReads = 0;
static float filteredAX = 0, filteredAY = 0, filteredAZ = 0;

static void controlTask(void*) {
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        MotorControl::updateEncoders();
        MotorControl::sampleEncoderPins();

        const uint32_t now = millis();
        portENTER_CRITICAL(&commandMux);
        sharedCommand.expire(now);
        const DriveCommand command = sharedCommand;
        portEXIT_CRITICAL(&commandMux);

        float leftPwm, rightPwm;
        drive.update(command, now, MotorControl::getLeftTicks(), MotorControl::getRightTicks(), leftPwm, rightPwm);
        MotorControl::setMotorSpeeds(leftPwm, rightPwm);
        leftPwmNow = leftPwm;
        rightPwmNow = rightPwm;
    }
}

// One write per line, so a line can't be split by a task switch. With no host reading, the USB
// driver throws away its oldest data, after waiting at most 100 ms the first time.
static void sendFormatted(const char* format, ...) {
    char line[160];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length > 0 && length < (int)sizeof(line)) {
        Serial.write((const uint8_t*)line, length);
    }
}

static bool beginImu() {
    if (!mpu.begin()) {
        return false;
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    return true;
}

static void sendEncoderLine() {
    sendFormatted("e %ld %ld\r\n", (long)MotorControl::getLeftTicks(), (long)MotorControl::getRightTicks());
}

// getEvent doesn't report a failed read. It gives garbage instead, and a failed read of the range
// setting scales everything 8 times too large. So check the IMU answers, and that the values fit
// the range it's set to.
static bool readImu(sensors_event_t& a, sensors_event_t& g) {
    Wire.beginTransmission(MPU6050_I2CADDR_DEFAULT);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    sensors_event_t temp;
    mpu.getEvent(&a, &g, &temp);
    return fabsf(a.acceleration.x) <= IMU_MAX_ACCEL && fabsf(a.acceleration.y) <= IMU_MAX_ACCEL &&
           fabsf(a.acceleration.z) <= IMU_MAX_ACCEL && fabsf(g.gyro.x) <= IMU_MAX_GYRO &&
           fabsf(g.gyro.y) <= IMU_MAX_GYRO && fabsf(g.gyro.z) <= IMU_MAX_GYRO;
}

static void sendImuLine() {
    sensors_event_t a, g;
    if (!readImu(a, g)) {
        imuBadReads++;
        if (++imuBadInARow >= IMU_MAX_BAD_READS) {
            imuReady = false;  // stops the `i` lines, shows red, and retries while idle
            imuBadInARow = 0;
        }
        return;
    }
    imuBadInARow = 0;

    filteredAX = (IMU_LPF_ALPHA * a.acceleration.x) + ((1.0f - IMU_LPF_ALPHA) * filteredAX);
    filteredAY = (IMU_LPF_ALPHA * a.acceleration.y) + ((1.0f - IMU_LPF_ALPHA) * filteredAY);
    filteredAZ = (IMU_LPF_ALPHA * a.acceleration.z) + ((1.0f - IMU_LPF_ALPHA) * filteredAZ);

    sendFormatted("i %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
                  filteredAX, filteredAY, filteredAZ, g.gyro.x, g.gyro.y, g.gyro.z);
}

// `g` names. These are the values in the "set from a measurement" block of config.h.
struct TuningEntry {
    const char* name;
    float* value;
};
static const TuningEntry TUNING_ENTRIES[] = {
    {"dead_l", &tuning.left.deadbandPwm},   {"dead_r", &tuning.right.deadbandPwm},
    {"slope_l", &tuning.left.slopePwmPerMps}, {"slope_r", &tuning.right.slopePwmPerMps},
    {"kick_l", &tuning.left.kickPwm},       {"kick_r", &tuning.right.kickPwm},
    {"kick_ms", &tuning.kickMs},            {"kp", &tuning.kp},
    {"ki", &tuning.ki},                     {"trim", &tuning.trimMaxPwm},
    {"cpr", &tuning.countsPerTurn},
};

static const float TUNING_MAX = 100000;  // keeps every printed value short enough for printTuning

static void printTuning() {
    char line[256];
    int n = snprintf(line, sizeof(line), "g closed %d", tuning.closedLoop ? 1 : 0);
    for (const TuningEntry& entry : TUNING_ENTRIES) {
        n += snprintf(line + n, sizeof(line) - n, " %s %.1f", entry.name, *entry.value);
    }
    n += snprintf(line + n, sizeof(line) - n, "\r\n");
    Serial.write((const uint8_t*)line, n);
}

static bool setTuning(const char* name, float value) {
    if (strcmp(name, "closed") == 0) {
        tuning.closedLoop = value != 0;
        return true;
    }
    for (const TuningEntry& entry : TUNING_ENTRIES) {
        if (strcmp(name, entry.name) == 0) {
            const bool needsPositive = strcmp(name, "cpr") == 0 || strcmp(name, "trim") == 0;
            if (value < (needsPositive ? 1 : 0) || value > TUNING_MAX) {
                return false;
            }
            *entry.value = value;
            return true;
        }
    }
    return false;
}

static void printWheel(const char* name, const WheelController& wheel, int32_t count, float pwm) {
    sendFormatted("d %s count=%ld target=%.3f speed=%.3f pwm=%.0f ff=%.0f trim=%.0f feedback=%d fault=%s faults=%lu\r\n",
                  name, (long)count, wheel.targetMps(), wheel.measuredMps(), pwm,
                  wheel.feedForwardPwm(), wheel.trimPwm(), wheel.feedbackActive() ? 1 : 0,
                  encoderFaultName(wheel.fault()), (unsigned long)wheel.faultCount());
}

static void printDiagnostics() {
    printWheel("L", drive.left(), MotorControl::getLeftTicks(), leftPwmNow);
    printWheel("R", drive.right(), MotorControl::getRightTicks(), rightPwmNow);

    // pin=level/transitions. Left A, left B, right A, right B, then the two old left pins.
    char line[128];
    int n = snprintf(line, sizeof(line), "d pins");
    for (uint8_t i = 0; i < MotorControl::WATCHED_PIN_COUNT; i++) {
        n += snprintf(line + n, sizeof(line) - n, " %u=%d/%lu", MotorControl::watchedPin(i),
                      MotorControl::watchedPinLevel(i), (unsigned long)MotorControl::watchedPinTransitions(i));
    }
    n += snprintf(line + n, sizeof(line) - n, "\r\n");
    Serial.write((const uint8_t*)line, n);

    sendFormatted("d encoders=%d imu=%d imu_bad_reads=%lu uptime_ms=%lu\r\n", encodersReady ? 1 : 0, imuReady ? 1 : 0,
                  (unsigned long)imuBadReads, (unsigned long)millis());
}

static void handleLine(const char* text) {
    const Command command = parseCommand(text);
    switch (command.type) {
        case CommandType::Velocity:
            portENTER_CRITICAL(&commandMux);
            sharedCommand.setVelocity(command.a, command.b, millis());
            portEXIT_CRITICAL(&commandMux);
            break;
        case CommandType::ForcePwm: {
            const int left = (int)constrain(command.a, (float)-PWM_MAX, (float)PWM_MAX);
            const int right = (int)constrain(command.b, (float)-PWM_MAX, (float)PWM_MAX);
            portENTER_CRITICAL(&commandMux);
            sharedCommand.setForce(left, right, millis());
            portEXIT_CRITICAL(&commandMux);
            sendFormatted("FORCE_PWM: %d %d\r\n", left, right);
            break;
        }
        case CommandType::Diagnostics:
            printDiagnostics();
            break;
        case CommandType::GetTuning:
            printTuning();
            break;
        case CommandType::SetTuning:
            if (setTuning(command.name, command.a)) {
                printTuning();
            }
            break;
        case CommandType::Invalid:
            break;
    }
}

static void pollSerial() {
    static char line[64];
    static size_t length = 0;
    static bool overflow = false;

    while (Serial.available()) {
        const char c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (length > 0 && !overflow) {
                line[length] = '\0';
                handleLine(line);
            }
            length = 0;
            overflow = false;
        } else if (length < sizeof(line) - 1) {
            line[length++] = c;
        } else {
            overflow = true;
        }
    }
}

static void reportFault(const char* name, EncoderFault now, EncoderFault& reported) {
    if (now != reported) {
        reported = now;
        if (now != EncoderFault::None) {
            sendFormatted("d fault %s %s\r\n", name, encoderFaultName(now));
        }
    }
}

static void showLed(uint32_t color) {
    static uint32_t shown = 0xFFFFFFFF;
    if (color != shown) {
        shown = color;
        pixels.setPixelColor(0, color);
        pixels.show();
    }
}

void setup() {
    encodersReady = MotorControl::begin();

    pixels.begin();
    pixels.setBrightness(30);
    showLed(pixels.Color(50, 50, 0));  // Yellow = Booting

    Serial.setRxBufferSize(1024);  // Increase buffer for stable command reception
    Serial.setTxBufferSize(1024);
    Serial.begin(115200);

    Wire.begin(IMU_SDA, IMU_SCL);
    imuReady = beginImu();

    xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, ARDUINO_RUNNING_CORE);

    sendFormatted("LLC_READY%s%s\r\n", encodersReady ? "" : " ENCODER_ERROR", imuReady ? "" : " IMU_ERROR");
}

void loop() {
    pollSerial();

    const uint32_t now = millis();

    static uint32_t lastEncoderLine = 0;
    if (now - lastEncoderLine >= TELEMETRY_PERIOD_MS) {
        lastEncoderLine = now - (now - lastEncoderLine) % TELEMETRY_PERIOD_MS;
        sendEncoderLine();
    }

    static uint32_t lastImuLine = 0;
    if (imuReady && now - lastImuLine >= TELEMETRY_PERIOD_MS) {
        lastImuLine = now - (now - lastImuLine) % TELEMETRY_PERIOD_MS;
        sendImuLine();
    }

    portENTER_CRITICAL(&commandMux);
    const DriveCommand command = sharedCommand;
    portEXIT_CRITICAL(&commandMux);
    const bool active = (command.hasVelocity && (int32_t)(now - command.velocityMs) < COMMAND_ACTIVE_MS) ||
                        command.forceActive(now);

    // beginImu() takes hundreds of ms, so only retry a missing IMU while no command is arriving.
    static uint32_t lastImuTry = 0;
    if (!imuReady && !active && now - lastImuTry >= IMU_RETRY_MS) {
        lastImuTry = now;
        imuReady = beginImu();
    }

    static EncoderFault reportedLeft = EncoderFault::None;
    static EncoderFault reportedRight = EncoderFault::None;
    reportFault("L", drive.left().fault(), reportedLeft);
    reportFault("R", drive.right().fault(), reportedRight);

    if (!imuReady) {
        showLed(pixels.Color(50, 0, 0));  // Red = IMU not found
    } else if (reportedLeft != EncoderFault::None || reportedRight != EncoderFault::None) {
        showLed(pixels.Color(60, 20, 0));  // Orange = a wheel is on feed-forward alone
    } else if (active) {
        showLed(pixels.Color(0, 0, 150));  // Blue = Receiving commands
    } else {
        showLed(pixels.Color(0, 50, 0));  // Green = Ready
    }

    delay(1);
}
