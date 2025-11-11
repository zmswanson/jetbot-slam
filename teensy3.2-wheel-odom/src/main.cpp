#include <Arduino.h>
#include "WheelEncoders.h"

// Pin assignments for DFRobot FIT0450 encoders
#define LEFT_A 2
#define LEFT_B 3
#define RIGHT_A 4
#define RIGHT_B 5

WheelEncoders encoders(LEFT_A, LEFT_B, RIGHT_A, RIGHT_B);

void setup() {
    Serial.begin(115200);
    encoders.begin();
}

void loop() {
    if (encoders.update()) {
        Serial.println(encoders.toJSON());
    }
}
