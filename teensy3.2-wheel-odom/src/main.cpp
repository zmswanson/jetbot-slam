#include <Arduino.h>
#include "WheelEncoders.h"

// Pin assignments for DFRobot FIT0450 encoders
#define LEFT_A 2
#define LEFT_B 3
#define RIGHT_A 4
#define RIGHT_B 5

WheelEncoders encoders(LEFT_A, LEFT_B, RIGHT_A, RIGHT_B);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial1.begin(115200);
    
    // // Wait for Serial connection or timeout after 5 seconds
    while (!Serial1 && millis() < 5000) { delay(10); }

    encoders.begin();
}

void loop() {
    static auto led_count = 0;

    if (encoders.update()) {
        Serial1.println(encoders.toJSON());

        if ((++led_count % 10) == 0) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
    }
}
