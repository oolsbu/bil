#include "bil.h"
#include <Arduino.h>
int interruptPin1 = 2;
int interruptPin2 = 3;
volatile int rightWheelCount = 0;
volatile int leftWheelCount = 0;

// Motor A
int IN1 = 8;
int IN2 = 7;
int ENA = 9;

// Motor B
int IN3 = 5;
int IN4 = 4;
int ENB = 10;

// photoresistors
int lightRight = A0;
int lightLeft = A1;

void trigLeftWheel (){
    leftWheelCount++;
}

void trigRightWheel (){
    rightWheelCount++;
}

void runLeft(int speed, boolean drive, boolean direction){
    if(!drive){
        digitalWrite(ENA, LOW);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        Serial.println("Left Motor stopped running!");
        return;
    }
    switch(direction){
        case true:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, speed);
            Serial.println("Left Motor Switching!");
            break;
        case false:
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, speed);
            Serial.println("Left Motor running!");
            break;      
    }
}
void runRight(int speed, boolean drive, boolean direction){
    if(!drive){
        digitalWrite(ENB, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        Serial.println("Right Motor stopped running!");
        return;
    }
    switch(direction){
        case true:
            digitalWrite(IN3, LOW);
            digitalWrite(4, HIGH);
            analogWrite(ENB, speed);
            Serial.println("Right Motor switching!");
            break;
        case false:
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENB, speed);
            Serial.println("Right Motor running!");
            break;      
    }
}

void setup() {
    pinMode(interruptPin1, INPUT_PULLUP);
    pinMode(interruptPin2, INPUT_PULLUP);
    // Initiate motor A pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    // Initiate motor B pins
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    // Initiate light resistor pins
    pinMode(lightLeft, INPUT);
    pinMode(lightRight, INPUT);
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(interruptPin1), trigLeftWheel, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptPin2), trigRightWheel, CHANGE);
}


void loop() {
    runLeft(map(0, 1023, 0, 255, analogRead(lightLeft)), true, true);
    runRight(map(0, 1023, 0, 255, analogRead(lightRight)), true, true);
    Serial.print("light left: ");
    Serial.print(analogRead(lightLeft));
    Serial.print(" light right: ");
    Serial.println(analogRead(lightRight));
    delay(100);
}

