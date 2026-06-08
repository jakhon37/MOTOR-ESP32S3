// MOTOR-ESP32S3/src/main.cpp - STABLE PRODUCTION
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "MotorControl.h"

Adafruit_NeoPixel pixels(1, 48, NEO_GRB + NEO_KHZ800);
float targetLeftVel = 0, targetRightVel = 0;
uint32_t lastCmdTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.setRxBufferSize(1024); // Increase buffer for stable command reception
    pixels.begin();
    pixels.setBrightness(30);
    pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green = Ready
    pixels.show();
    
    MotorControl::begin();
    Serial.println("LLC_READY");
}

void loop() {
    // 1. Handle Serial Commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'm') {
            targetLeftVel = Serial.parseFloat();
            targetRightVel = Serial.parseFloat();
            lastCmdTime = millis();
            
            // Pulse Blue on command
            pixels.setPixelColor(0, pixels.Color(0, 0, 150));
            pixels.show();
        } else if (cmd == 'f') { // Force PWM Test
            int p = Serial.parseInt();
            Serial.print("FORCE_PWM: "); Serial.println(p);
            MotorControl::setMotorSpeeds(p, p);
            delay(2000);
            MotorControl::setMotorSpeeds(0, 0);
        }
    }

    // 2. Motor Logic (Aggressive scaling for 5V systems)
    if (millis() - lastCmdTime < CMD_TIMEOUT) {
        float outL = targetLeftVel * 2000.0; 
        float outR = targetRightVel * 2000.0;
        
        // Kickstart: Ensure we have enough power to overcome friction
        if (targetLeftVel > 0.01 && outL < 500) outL = 500;
        if (targetLeftVel < -0.01 && outL > -500) outL = -500;
        if (targetRightVel > 0.01 && outR < 500) outR = 500;
        if (targetRightVel < -0.01 && outR > -500) outR = -500;

        MotorControl::setMotorSpeeds(outL, outR);
    } else {
        MotorControl::setMotorSpeeds(0, 0);
        
        // Return to Green if no recent command
        if (millis() - lastCmdTime > 200) {
            pixels.setPixelColor(0, pixels.Color(0, 50, 0));
            pixels.show();
        }
    }

    // 3. Telemetry - SLOWER (20Hz) to prevent Mac socat congestion
    static uint32_t lastTele = 0;
    if (millis() - lastTele > 50) {
        lastTele = millis();
        Serial.print("e ");
        Serial.print(MotorControl::getLeftTicks());
        Serial.print(" ");
        Serial.println(MotorControl::getRightTicks());
    }
}
