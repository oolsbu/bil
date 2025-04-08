#include <Arduino.h>

// Pulse counter pins
int interruptPinLeft = 2;
int interruptPinRight = 3;

// Defining pulse counters
volatile int rightPulseCount = 0;
volatile int leftPulseCount = 0;

// Defining statistics
int rightWheelRotations = 0;
int leftWheelRotations = 0;
int rightWheelSpeed = 0;
int leftWheelSpeed = 0;

// light sensor PID
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.0;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

// Motor A
int IN1 = 8;
int IN2 = 7;
int ENA = 9;

// Motor B
int IN3 = 5;
int IN4 = 4;
int ENB = 10;

// photoresistors
int lightRight = 0;
int lightLeft = 0;


void read_sensors() {

}

void calculate_error() {

}

void update_pid() {

}

void setup() {

}

void loop(){
    int readLightRight = analogRead(A0);
    int readLightLeft = analogRead(A1);
    Serial.print(readLightLeft);

    error = readLightLeft - readLightRight;
}
