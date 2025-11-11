#include "WheelEncoders.h"

WheelEncoders::WheelEncoders(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB,
                             float countsPerRev, unsigned long samplePeriodMs)
    : leftEncoder(leftA, leftB),
      rightEncoder(rightA, rightB),
      countsPerRev(countsPerRev),
      samplePeriodMs(samplePeriodMs),
      lastUpdate(0),
      prevLeftCount(0),
      prevRightCount(0),
      rpmLeft(0.0f),
      rpmRight(0.0f)
{}

void WheelEncoders::begin() {
    leftEncoder.write(0);
    rightEncoder.write(0);
    prevLeftCount = 0;
    prevRightCount = 0;
    lastUpdate = millis();
}

bool WheelEncoders::update() {
    unsigned long now = millis();
    if (now - lastUpdate < samplePeriodMs)
        return false;

    long leftCount = leftEncoder.read();
    long rightCount = rightEncoder.read();

    long deltaLeft = leftCount - prevLeftCount;
    long deltaRight = rightCount - prevRightCount;

    float revLeft = deltaLeft / countsPerRev;
    float revRight = deltaRight / countsPerRev;

    rpmLeft = revLeft * (60000.0 / samplePeriodMs);
    rpmRight = revRight * (60000.0 / samplePeriodMs);

    prevLeftCount = leftCount;
    prevRightCount = rightCount;
    lastUpdate = now;
    return true;
}

String WheelEncoders::toJSON() const {
    String json = "{\"rpm_left\":";
    json += String(rpmLeft, 2);
    json += ",\"rpm_right\":";
    json += String(rpmRight, 2);
    json += "}";
    return json;
}
