#pragma once

#include <Arduino.h>
#include <Motor.h>
#include <Encoder.h>

class MotorWithEncoder {
public:
    MotorWithEncoder(int in1, int in2, int pwm, int eA, int eB, bool reversed=false);
    void begin();
    void setSpeed(int speed);
    long getEncoderCount();
    void resetEncoder();

private:
    Motor   _motor;
    Encoder _encoder;
};
