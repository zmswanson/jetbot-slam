#include <Arduino.h>

#define ENC1_A 2
#define ENC1_B 3
#define ENC2_A 4
#define ENC2_B 5

volatile long count1 = 0;
volatile long count2 = 0;

void IRAM_ATTR encoder1ISR() {
  bool A = digitalRead(ENC1_A);
  bool B = digitalRead(ENC1_B);
  count1 += (A == B) ? 1 : -1;
}
void IRAM_ATTR encoder2ISR() {
  bool A = digitalRead(ENC2_A);
  bool B = digitalRead(ENC2_B);
  count2 += (A == B) ? 1 : -1;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), encoder2ISR, CHANGE);
}

void loop() {
  static unsigned long last = millis();
  unsigned long now = millis();
  if (now - last >= 100) {  // every 100 ms
    noInterrupts();
    long c1 = count1;
    long c2 = count2;
    count1 = 0;
    count2 = 0;
    interrupts();

    // 1920 counts = 1 rev
    float rev1 = c1 / 1920.0;
    float rev2 = c2 / 1920.0;
    float rpm1 = rev1 * 600.0;  // 100 ms → ×10 ×60
    float rpm2 = rev2 * 600.0;

    Serial.printf("{\"rpm_left\":%.2f,\"rpm_right\":%.2f}\n", rpm1, rpm2);
    last = now;
  }
}
