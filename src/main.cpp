#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>

#include "hal/ServoHandler.h"
#include "hal/ESPNowHandler.h"
#include "hal/MotorDrive.h"
#include "MotionLogic.h"

ServoHandler legServos;
ESPNowHandler radio;
MotorDrive motors;
MotionLogic logic;

// Directional Button Mask (Bits 0-3: UP, DOWN, LEFT, RIGHT)
#define DIRECTIONAL_BTNS_MASK 0x0F

// Deadband tracking
float lastHeightOffset = -999.0f;
float lastSplitOffset  = -999.0f;
float lastNeckPM       = -999.0f;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== Quadruped System Initializing ===");

    // 1. WiFi & ESP-NOW Channel Lock
    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // 2. Shared I2C Bus for PCA9685 Servos
    Wire.begin(21, 22);
    Wire.setClock(100000);

    // 3. Initialize Hardware Modules
    legServos.begin();
    legServos.setNeutralPose();
    radio.begin();
    motors.init();

    Serial.println(">>> System Ready.");
}

void loop() {
    // 1. Interleaved non-blocking servo movement updates
    legServos.update();

    // 2. Process USB Serial keyboard inputs
    legServos.processSerialCommands();

    // 3. Process Wireless ESP-NOW Remote Commands
    if (radio.isConnected()) {
        const PacketData& remote = radio.getLatestPacket();

        // Check if directional buttons (bits 0-3) or joysticks are active
        bool isNavBtnPressed  = (remote.buttons & DIRECTIONAL_BTNS_MASK) != 0;
        bool isJoystickMoving = (abs(remote.rx) > 0.12f || abs(remote.ry) > 0.12f);
        bool isMoving         = isJoystickMoving || isNavBtnPressed;

        // --- A. MECANUM WHEEL DRIVE LOGIC ---
        MotionLogic::CalculatedSpeeds speeds = logic.process(remote);
        motors.drive(speeds.fl, speeds.fr, speeds.rl, speeds.rr);

        // --- B. LEG POSE CONTROL (pL: Height, pR: Split) ---
        float rawPL = remote.pL; // Height
        float rawPR = remote.pR; // Split

        // Deadband for pL (-0.20 to +0.20 inclusive)
        float targetHeightOffset = 0.0f;
        if (abs(rawPL) > 0.20f) {
            targetHeightOffset = round(rawPL * 40.0f);
        }

        // Deadband for pR (-0.20 to +0.20 inclusive)
        float targetSplitOffset = 0.0f;
        if (abs(rawPR) > 0.20f) {
            targetSplitOffset = round(rawPR * 40.0f);
        }

        // Dispatch pose offset updates when targets change
        if (targetHeightOffset != lastHeightOffset || targetSplitOffset != lastSplitOffset) {
            lastHeightOffset = targetHeightOffset;
            lastSplitOffset  = targetSplitOffset;

            legServos.applyPoseOffsets(targetHeightOffset, targetSplitOffset);

            Serial.printf(">>> REMOTE POSE: Height = %+.0f deg | Split = %+.0f deg\n", 
                          targetHeightOffset, targetSplitOffset);
        }

        // --- C. HEAD / NECK PAN CONTROL (pM: Middle Potentiometer) ---
        float rawPM = remote.pM;
        float targetPM = 0.0f;
        const float PM_DEADZONE = 0.20f; // [-0.20, +0.20] deadzone

        if (!isMoving && abs(rawPM) > PM_DEADZONE) {
            // Smoothly normalize range outside deadzone to (0.0 to 1.0)
            float normalized = (abs(rawPM) - PM_DEADZONE) / (1.0f - PM_DEADZONE);
            float sign = (rawPM > 0.0f) ? 1.0f : -1.0f;

            // Scale to 30 deg <-> 150 deg mapping
            targetPM = sign * normalized * (60.0f / 90.0f);
        }

        if (targetPM != lastNeckPM) {
            lastNeckPM = targetPM;
            legServos.setNeckPan(targetPM);

            float physicalAngle = 90.0f - (targetPM * 90.0f);
            Serial.printf(">>> NECK PAN: pM_out = %+.3f | Physical Angle = %.1f deg\n", 
                          targetPM, physicalAngle);
        }

        // --- D. WEAPON ENCODERS (DUAL CONTROL VIA RIGHT ENCODER) ---
        if (isMoving) {
            legServos.setIndividualWeapons(0, 0); // Interlock: Cut PWM while chassis moves
        } else {
            // Both Gatling (encL param) and Drill (encR param) receive remote.encR
            legServos.setIndividualWeapons(remote.encR, remote.encR);
        }

    } else {
        // Safety stop if remote disconnects
        motors.stop();
        legServos.setNeckPan(0.0f); // Return to 90 deg center
        legServos.setIndividualWeapons(0, 0);
    }
}