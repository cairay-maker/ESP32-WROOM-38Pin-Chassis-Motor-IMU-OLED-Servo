#include "MotionLogic.h"

// Controller Bitmask Definitions
#define BTN_UP_MASK     (1 << 0) // Bit 0: UP
#define BTN_DOWN_MASK   (1 << 1) // Bit 1: DOWN
#define BTN_LEFT_MASK   (1 << 2) // Bit 2: LEFT
#define BTN_RIGHT_MASK  (1 << 3) // Bit 3: RIGHT
#define BTN_SELECT_MASK (1 << 4) // Bit 4: SELECT

MotionLogic::MotionLogic() {}

MotionLogic::CalculatedSpeeds MotionLogic::process(const PacketData& joyData) {
    CalculatedSpeeds s = {0, 0, 0, 0};

    // Extract bitwise button states
    bool isUpHeld    = (joyData.buttons & BTN_UP_MASK) != 0;
    bool isDownHeld  = (joyData.buttons & BTN_DOWN_MASK) != 0;
    bool isLeftHeld  = (joyData.buttons & BTN_LEFT_MASK) != 0;
    bool isRightHeld = (joyData.buttons & BTN_RIGHT_MASK) != 0;

    // --- 1. PRE-PROGRAMMED BUTTON MANEUVERS (50% Max Cap) ---

    // LEFT BUTTON: Stationary Front-Left Pivot (FL locked at 0)
    if (isLeftHeld) {
        s.fl = 0;    // FL wheel stationary (Pivot anchor)
        s.fr = 84;   // Inner wheel
        s.rl = -84;  // Inner wheel
        s.rr = 127;  // Outer wheel (50% max speed)
        return s;
    }

    // UP BUTTON: Active Counter-Clockwise Arc
    if (isUpHeld) {
        s.fl = -84;  
        s.fr = 84;   
        s.rl = -84;  
        s.rr = 127;  
        return s;
    }

    // RIGHT BUTTON: Stationary Front-Right Pivot (FR locked at 0)
    if (isRightHeld) {
        s.fl = 84;   
        s.fr = 0;    // FR wheel stationary (Pivot anchor)
        s.rl = 127;  // Outer wheel (50% max speed)
        s.rr = -84;  
        return s;
    }

    // DOWN BUTTON: Reverse Pivot Arc
    if (isDownHeld) {
        s.fl = -84;  
        s.fr = -84;  
        s.rl = 127;  
        s.rr = 127;  
        return s;
    }

    // --- 2. STANDARD JOYSTICK DRIVING ---
    float rawX = joyData.rx * 500.0f; 
    float rawY = joyData.ry * 500.0f;

    // Apply Deadzone
    if (abs(rawX) <= DEAD_ZONE) rawX = 0;
    if (abs(rawY) <= DEAD_ZONE) rawY = 0;

    // Apply 50% speed scaling factor (SCALE = 127.0 / 500.0)
    float x = rawX * SCALE;
    float y = rawY * SCALE;

    // INNER ZONE: Forward/Backward or Point Turns
    if (abs(rawX) <= INNER_ZONE_LIMIT && abs(rawY) <= INNER_ZONE_LIMIT) {
        if (abs(rawY) > DEAD_ZONE) {
            s.fl = s.fr = s.rl = s.rr = (int)y;
        } 
        else if (abs(rawX) > DEAD_ZONE) {
            // Point Turn Logic
            s.fl = s.rl = (int)x;
            s.fr = s.rr = (int)-x;
        }
    } 
    // OUTER ZONE: Full Mecanum Strafe movement
    else {
        float rot = (abs(rawY) > DEAD_ZONE) ? 0 : x * 0.2f;
        s.fl = (int)(y - x + rot);
        s.fr = (int)(y + x - rot);
        s.rl = (int)(y + x + rot);
        s.rr = (int)(y - x - rot);
    }

    // Hard safety clamp at max speed 127 (50% PWM)
    s.fl = constrain(s.fl, -127, 127);
    s.fr = constrain(s.fr, -127, 127);
    s.rl = constrain(s.rl, -127, 127);
    s.rr = constrain(s.rr, -127, 127);

    return s;
}