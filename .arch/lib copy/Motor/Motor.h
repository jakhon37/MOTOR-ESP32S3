#pragma once
#include <Arduino.h>

class Motor {
public:
    Motor(int pinIn1, int pinIn2, int pinPwm, bool reversed=false);

    void begin();
    void setSpeed(int speed); // range -255..255 or scaled as needed

private:
    int _in1, _in2, _pwm;
    bool _reversed;
};
