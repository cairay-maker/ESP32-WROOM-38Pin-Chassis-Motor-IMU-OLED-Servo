#include "hal/ServoHandler.h"

#define SERVOMIN 100
#define SERVOMAX 500

ServoHandler::ServoHandler() : _pwm(Adafruit_PWMServoDriver(0x40)) {}

uint16_t ServoHandler::degreesToPWM(float degrees) {
    degrees = constrain(degrees, 0.0f, 180.0f);
    return map((long)(degrees * 10), 0, 1800, SERVOMIN, SERVOMAX);
}

void ServoHandler::begin() {
    _pwm.begin();
    _pwm.setOscillatorFrequency(27000000);
    _pwm.setPWMFreq(50);
    _lastUpdateMicros = micros();
}

void ServoHandler::setEngaged(bool engage) {
    _isEngaged = engage;
    if (_isEngaged) {
        for (int i = 0; i < LEG_JOINT_COUNT; i++) {
            _targetAngles[i] = _currentAngles[i];
            _pwm.setPWM(i, 0, degreesToPWM(_currentAngles[i]));
        }
        Serial.println(">>> SERVOS: ENGAGED");
    } else {
        for (int i = 0; i < LEG_JOINT_COUNT; i++) {
            _pwm.setPWM(i, 0, 0); // De-energize leg PWM outputs
        }
        // Completely stop and cut PWM output to continuous weapon servos
        _pwm.setPWM(WEAPON_GATLING, 0, 0);
        _pwm.setPWM(WEAPON_DRILL, 0, 0);
        Serial.println(">>> SERVOS: RELAXED");
    }
}

void ServoHandler::setNeutralPose() {
    for (int i = 0; i < LEG_JOINT_COUNT; i++) {
        _targetAngles[i] = NEUTRAL_ANGLES[i];
    }
    setNeckPan(0.0f);
    setIndividualWeapons(0, 0);
    Serial.println(">>> POSE: NEUTRAL RESTORED");
}

void ServoHandler::adjustHeight(float offsetDeg) {
    _targetAngles[JOINT_FL] = constrain(NEUTRAL_ANGLES[JOINT_FL] - offsetDeg, 0.0f, 180.0f);
    _targetAngles[JOINT_RR] = constrain(NEUTRAL_ANGLES[JOINT_RR] - offsetDeg, 0.0f, 180.0f);
    _targetAngles[JOINT_FR] = constrain(NEUTRAL_ANGLES[JOINT_FR] + offsetDeg, 0.0f, 180.0f);
    _targetAngles[JOINT_RL] = constrain(NEUTRAL_ANGLES[JOINT_RL] + offsetDeg, 0.0f, 180.0f);
}

void ServoHandler::applySplit(float offsetDeg) {
    applyPoseOffsets(0.0f, offsetDeg);
}

void ServoHandler::applyPoseOffsets(float heightOffsetDeg, float splitOffsetDeg) {
    if (!_isEngaged) {
        setEngaged(true);
    }

    // 1. Base height targets (pL)
    float flTarget = NEUTRAL_ANGLES[JOINT_FL] - heightOffsetDeg;
    float rrTarget = NEUTRAL_ANGLES[JOINT_RR] - heightOffsetDeg;
    float frTarget = NEUTRAL_ANGLES[JOINT_FR] + heightOffsetDeg;
    float rlTarget = NEUTRAL_ANGLES[JOINT_RL] + heightOffsetDeg;

    // 2. Direct single front leg lift (pR)
    if (splitOffsetDeg < 0.0f) {
        // Negative pR: Raises Front-Left (FL) leg only
        flTarget += splitOffsetDeg;
    } else if (splitOffsetDeg > 0.0f) {
        // Positive pR: Raises Front-Right (FR) leg only
        frTarget += splitOffsetDeg;
    }

    // 3. Constrain and set target angles
    _targetAngles[JOINT_FL] = constrain(flTarget, 0.0f, 180.0f);
    _targetAngles[JOINT_RR] = constrain(rrTarget, 0.0f, 180.0f);
    _targetAngles[JOINT_FR] = constrain(frTarget, 0.0f, 180.0f);
    _targetAngles[JOINT_RL] = constrain(rlTarget, 0.0f, 180.0f);
}

// --- NECK PAN (Controlled by pM on Pin 8 - Inverted Direction) ---
void ServoHandler::setNeckPan(float rawPM) {
    if (abs(rawPM) < 0.05f) rawPM = 0.0f;

    float targetAngle = 90.0f - (rawPM * 90.0f);
    targetAngle = constrain(targetAngle, 0.0f, 180.0f);

    _pwm.setPWM(NECK_PAN, 0, degreesToPWM(targetAngle));
}

// --- INDIVIDUAL WEAPON ENCODER CONTROL (Pins 9 & 10) ---
void ServoHandler::setIndividualWeapons(int16_t encL, int16_t encR) {
    encL = constrain(encL, -3, 3);
    encR = constrain(encR, -3, 3);

    // 1. Gatling Gun (encL)
    if (encL == 0) {
        _pwm.setPWM(WEAPON_GATLING, 0, 0); // Cut PWM signal completely when stopped
    } else {
        float gatlingAngle = 90.0f + (encL * 2.5f);
        _pwm.setPWM(WEAPON_GATLING, 0, degreesToPWM(gatlingAngle));
    }

    // 2. Drill Motor (encR)
    if (encR == 0) {
        _pwm.setPWM(WEAPON_DRILL, 0, 0); // Cut PWM signal completely when stopped
    } else {
        float drillAngle = 90.0f + (encR * 2.5f);
        _pwm.setPWM(WEAPON_DRILL, 0, degreesToPWM(drillAngle));
    }
}

void ServoHandler::processSerialCommands() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    if (input.length() == 0) return;

    if (input == "OFF") {
        setEngaged(false);
    } else if (input == "ON") {
        setEngaged(true);
    } else if (input == "NEUTRAL") {
        setNeutralPose();
    } else if (input.indexOf(':') > 0) {
        String key = input.substring(0, input.indexOf(':'));
        float val = input.substring(input.indexOf(':') + 1).toFloat();

        if (!_isEngaged) setEngaged(true);

        if (key == "UP") {
            adjustHeight(val);
        } else if (key == "DN") {
            adjustHeight(-val);
        } else if (key == "SPLIT") {
            applySplit(val);
        } else if (key == "NECK") {
            setNeckPan(val);
        } else if (key == "SPEED") {
            _speedDegPerSec = constrain(val, 5.0f, 300.0f);
            Serial.printf(">>> Speed set to %.1f deg/sec\n", _speedDegPerSec);
        } else {
            int ch = -1;
            if (key == "FL") ch = JOINT_FL;
            else if (key == "FR") ch = JOINT_FR;
            else if (key == "RL") ch = JOINT_RL;
            else if (key == "RR") ch = JOINT_RR;

            if (ch >= 0) {
                _targetAngles[ch] = constrain(val, 0.0f, 180.0f);
                Serial.printf(">>> Joint %s target set to %.1f deg\n", key.c_str(), val);
            } else {
                Serial.println("!!! Unknown command. Type HELP for options.");
            }
        }
    } else {
        Serial.println("!!! Unknown command. Type HELP for options.");
    }
}

void ServoHandler::update() {
    unsigned long currentMicros = micros();
    float dt = (currentMicros - _lastUpdateMicros) / 1000000.0f;
    _lastUpdateMicros = currentMicros;

    if (!_isEngaged) return;

    float maxStep = _speedDegPerSec * dt;

    for (int count = 0; count < LEG_JOINT_COUNT; count++) {
        int leg = (_interleavedLegIndex + count) % LEG_JOINT_COUNT;
        float diff = _targetAngles[leg] - _currentAngles[leg];

        if (abs(diff) > 0.05f) {
            float step = (diff > 0) ? min(1.0f, maxStep) : max(-1.0f, -maxStep);
            _currentAngles[leg] += step;

            if ((diff > 0 && _currentAngles[leg] > _targetAngles[leg]) ||
                (diff < 0 && _currentAngles[leg] < _targetAngles[leg])) {
                _currentAngles[leg] = _targetAngles[leg];
            }

            _pwm.setPWM(leg, 0, degreesToPWM(_currentAngles[leg]));

            _interleavedLegIndex = (leg + 1) % LEG_JOINT_COUNT;
            break;
        }
    }
}