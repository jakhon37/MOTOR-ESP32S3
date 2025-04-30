#include "MotorWithEncoder.h"

MotorWithEncoder::MotorWithEncoder(int in1, int in2, int pwm, int eA, int eB, bool reversed)
    : _motor(in1, in2, pwm, reversed), _encoder(eA, eB)
{}

void MotorWithEncoder::begin() {
    _motor.begin();
    _encoder.begin();
}

void MotorWithEncoder::setSpeed(int speed) {
    _motor.setSpeed(speed);
}

long MotorWithEncoder::getEncoderCount() {
    return _encoder.getCount();
}

void MotorWithEncoder::resetEncoder() {
    _encoder.reset();
}
