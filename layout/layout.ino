#include <arduino.h>

void setup() {
    Serial.begin(115200);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
}

void loop() {
    Serial.println(analogRead(A0) + analogRead(A1));
    delay(100);
}