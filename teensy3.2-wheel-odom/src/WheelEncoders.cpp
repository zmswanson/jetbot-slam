#include "WheelEncoders.h"

WheelEncoders::WheelEncoders(uint8_t leftA, uint8_t leftB, 
                             uint8_t rightA, uint8_t rightB,
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

/** 
 * Initializes the encoders by resetting their counts and setting
 * the initial previous counts and last update time.
 */
void WheelEncoders::begin() {
    leftEncoder.write(0);
    rightEncoder.write(0);
    prevLeftCount = 0;
    prevRightCount = 0;
    lastUpdate = millis();
}

/**
 * Updates the RPM calculations if the sample period has elapsed. The encoder
 * classes handle critical section management internally if interrupts are used.
 * Returns true if an update was performed, false otherwise.
 * 
 * @return bool
 */
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

/**
 * Serializes the current revolutions per minute (RPM) to two decimal places for
 * both left and right encoders in JSON format (see return value).
 * 
 * @return String - {"rpm_left": <value>, "rpm_right": <value> }
 */
String WheelEncoders::toJSON() const {
    String json = "{\"rpm_left\":";
    json += String(rpmLeft, 2);
    json += ",\"rpm_right\":";
    json += String(rpmRight, 2);
    json += "}";
    return json;
}
