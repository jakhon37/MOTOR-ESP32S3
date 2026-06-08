// MOTOR-ESP32S3/lib/PIDController/PIDController.cpp
#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float out_min, float out_max)
    : m_kp(kp), m_ki(ki), m_kd(kd), m_min_out(out_min), m_max_out(out_max), m_integral(0), m_last_error(0) {}

float PIDController::compute(float target, float current, float dt) {
    float error = target - current;
    
    // Proportional
    float p_out = m_kp * error;
    
    // Integral with anti-windup (clamp before adding)
    m_integral += error * dt;
    float i_out = m_ki * m_integral;
    
    // Derivative
    float derivative = (error - m_last_error) / dt;
    float d_out = m_kd * derivative;
    
    m_last_error = error;
    
    // Total Output clamped to range
    float output = p_out + i_out + d_out;
    if (output > m_max_out) {
        output = m_max_out;
        m_integral -= error * dt; // Simple anti-windup
    } else if (output < m_min_out) {
        output = m_min_out;
        m_integral -= error * dt;
    }
    
    return output;
}

void PIDController::setTunings(float kp, float ki, float kd) {
    m_kp = kp; m_ki = ki; m_kd = kd;
}

void PIDController::reset() {
    m_integral = 0;
    m_last_error = 0;
}
