#ifndef SERVO_HANDLER_H
#define SERVO_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define LEG_JOINT_COUNT 4

// PCA9685 Pin Assignments
enum ServoChannel {
    JOINT_FL       = 0, // Leg Servos
    JOINT_FR       = 1,
    JOINT_RL       = 2,
    JOINT_RR       = 3,
    NECK_PAN       = 8,  // Head / Neck Servo (PCA Pin 8)
    WEAPON_GATLING = 9,  // Gatling Gun (PCA Pin 9)
    WEAPON_DRILL   = 10  // Drill (PCA Pin 10)
};

class ServoHandler {
private:
    Adafruit_PWMServoDriver _pwm;
    
    // Leg Neutral Angles
    const float NEUTRAL_ANGLES[LEG_JOINT_COUNT] = {130.0f, 100.0f, 110.0f, 122.0f};

    float _currentAngles[LEG_JOINT_COUNT] = {130.0f, 100.0f, 110.0f, 122.0f};
    float _targetAngles[LEG_JOINT_COUNT]  = {130.0f, 100.0f, 110.0f, 122.0f};
    
    float _speedDegPerSec = 30.0f;
    bool _isEngaged = false;
    unsigned long _lastUpdateMicros = 0;
    int _interleavedLegIndex = 0;

    uint16_t degreesToPWM(float degrees);

public:
    ServoHandler();
    void begin();
    void update();
    void processSerialCommands();
    void setNeutralPose();
    void adjustHeight(float offsetDeg);
    void setEngaged(bool engage);
    void applySplit(float offsetDeg);
    void applyPoseOffsets(float heightOffsetDeg, float splitOffsetDeg);

    // Auxiliary Servos
    void setNeckPan(float rawPM);                            // Middle Potentiometer (-1.0 to 1.0)
    void setIndividualWeapons(int16_t encL, int16_t encR);   // Encoders (-3 to +3)
};

#endif