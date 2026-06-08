// MOTOR-ESP32S3/lib/PIDController/PIDController.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float out_min, float out_max);
    
    // Set target and compute output
    float compute(float target, float current, float dt);
    
    // Update constants
    void setTunings(float kp, float ki, float kd);
    
    // Reset integral state
    void reset();

private:
    float m_kp, m_ki, m_kd;
    float m_min_out, m_max_out;
    float m_integral;
    float m_last_error;
};

#endif
