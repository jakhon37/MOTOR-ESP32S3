#include "Motor.h"

Motor::Motor(int pinIn1, int pinIn2, int pinPwm, bool reversed)
    : _in1(pinIn1), _in2(pinIn2), _pwm(pinPwm), _reversed(reversed)
{}

void Motor::begin() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_pwm, OUTPUT);

    // For ESP32/ESP32-C3, you may need to configure LEDC for PWM:
    // Example (12-bit resolution, 5kHz freq, channel 0):
    // ledcSetup(0, 5000, 12);
    // ledcAttachPin(_pwm, 0);
}

void Motor::setSpeed(int speed) {
    bool forward = (speed >= 0);
    int pwmVal = abs(speed);
    if (pwmVal > 255) pwmVal = 255;

    // If reversed, flip logic
    if (!_reversed) {
        digitalWrite(_in1, forward ? HIGH : LOW);
        digitalWrite(_in2, forward ? LOW : HIGH);
    } else {
        digitalWrite(_in1, forward ? LOW : HIGH);
        digitalWrite(_in2, forward ? HIGH : LOW);
    }

    // If using ledc:
    // ledcWrite(0, pwmVal);  // channel 0
    // If using standard Arduino (ESP8266 style):
    analogWrite(_pwm, pwmVal);
}
