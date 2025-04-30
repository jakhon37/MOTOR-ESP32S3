#include "Encoder.h"

Encoder::Encoder(int pinA, int pinB)
    : _pinA(pinA), _pinB(pinB), _count(0)
{}

void Encoder::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    // For ESP32-C3, we typically attach interrupt using an ISR that references a static function
    attachInterruptArg(digitalPinToInterrupt(_pinA), isrA, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_pinB), isrB, this, CHANGE);
}

long Encoder::getCount() {
    noInterrupts();
    long val = _count;
    interrupts();
    return val;
}

void Encoder::reset() {
    noInterrupts();
    _count = 0;
    interrupts();
}

void Encoder::handleInterrupt() {
    bool A = digitalRead(_pinA);
    bool B = digitalRead(_pinB);
    // basic quadrature logic:
    if (A == B) {
        _count++;
    } else {
        _count--;
    }
}

void Encoder::isrA(void* arg) {
    Encoder* enc = static_cast<Encoder*>(arg);
    enc->handleInterrupt();
}

void Encoder::isrB(void* arg) {
    Encoder* enc = static_cast<Encoder*>(arg);
    enc->handleInterrupt();
}
