#ifndef WHEEL_ENCODERS_H
#define WHEEL_ENCODERS_H

#include <Arduino.h>
#include <Encoder.h>

class WheelEncoders {
public:
    WheelEncoders(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB,
                  float countsPerRev = 1920.0, unsigned long samplePeriodMs = 100);

    void begin();
    bool update();
    String toJSON() const;

private:
    Encoder leftEncoder;
    Encoder rightEncoder;

    float countsPerRev;
    unsigned long samplePeriodMs;

    unsigned long lastUpdate;
    long prevLeftCount;
    long prevRightCount;
    float rpmLeft;
    float rpmRight;
};

#endif // WHEEL_ENCODERS_H