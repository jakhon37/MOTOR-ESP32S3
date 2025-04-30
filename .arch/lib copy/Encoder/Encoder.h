#pragma once
#include <Arduino.h>

class Encoder {
public:
    Encoder(int pinA, int pinB);
    void begin();
    long getCount();
    void reset();

private:
    int _pinA, _pinB;
    volatile long _count;

    void handleInterrupt();
    static void isrA(void* arg);
    static void isrB(void* arg);
};
