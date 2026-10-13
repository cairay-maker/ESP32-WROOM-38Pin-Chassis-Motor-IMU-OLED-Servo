#ifndef MOTION_LOGIC_H
#define MOTION_LOGIC_H

#include <Arduino.h>
#include "hal/ESPNowHandler.h"
#include "PacketData.h"

class MotionLogic {
public:
    struct CalculatedSpeeds {
        int fl, fr, rl, rr;
    };

    MotionLogic();
    
    CalculatedSpeeds process(const PacketData& joyData);

private:
    const int DEAD_ZONE = 10;
    const int INNER_ZONE_LIMIT = 495;
    
    // Scale factor for 30% max speed cap (76 PWM out of 255)
    const float SCALE = 127.0f / 500.0f;
};

#endif