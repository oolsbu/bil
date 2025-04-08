#include "bil.h"
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
float Kp = KP_VERDI;
float Ki = KI_VERDI;
float Kd = KD_VERDI;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
unsigned long previousTime = 0;

// Motor A
int IN1 = 8;
int IN2 = 7;
int ENA = 9;

// Motor B
int IN3 = 5;
int IN4 = 4;
int ENB = 10;

// photoresistors
int lightRightPin = A1;
int readLightRight = 0;
int lightLeftPin = A0;
int readLightLeft = 0;

void trigLeftWheel (){
    leftPulseCount++;
}

void trigRightWheel (){
    rightPulseCount++;
}

// float calculateDistance(int readLeft, int readRight){
//     int minste = min(readLeft,readRight);
//    return (0.0927611617 * minste - 0.7325530993);
// }

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
            Serial.println("Left Motor running clockwise!");
            break;
        case false:
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, speed);
            Serial.println("Left Motor running counter-clockwise!");
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
            Serial.println("Right Motor running clockwise!");
            break;
        case false:
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            analogWrite(ENB, speed);
            Serial.println("Right Motor counter-clockwise!");
            break;      
    }
}

void read_LightSensors() {
    // Reads light sensors
    int readLightRight = analogRead(lightRightPin);
    int readLightLeft = analogRead(lightLeftPin);
}

void calculate_pid() {
    // Read light sensor values
    int lightLeft = analogRead(lightLeftPin);
    int lightRight = analogRead(lightRightPin);

    // Calculate error (difference between left and right light intensity)
    error = lightLeft - lightRight;

    // Calculate time difference
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
    previousTime = currentTime;

    // Calculate integral (accumulated error over time)
    integral += error * deltaTime;

    // Calculate derivative (rate of change of error)
    derivative = (error - previousError) / deltaTime;

    // Calculate PID output
    float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Update previous error
    previousError = error;

    // Use the PID output to adjust motor speeds
    int baseSpeed = 150; // Base motor speed
    int leftMotorSpeed = constrain(baseSpeed + pidOutput, 0, 255);
    int rightMotorSpeed = constrain(baseSpeed - pidOutput, 0, 255);

    // Run motors with adjusted speeds
    runLeft(leftMotorSpeed, true, true);
    runRight(rightMotorSpeed, true, true);
}


float calculateArcWheelSpeed(boolean outer, int radius, int wheelDistance, int wantedSpeed){
return outer ? (wantedSpeed * (radius) + (wheelDistance / 2)): (wantedSpeed * (radius) - (wheelDistance / 2));
}

double calculateRPS(int counter){
    const int numberOfSlits = 20;
    const int oscillatorFrequency = 1;
    int N = counter;

    double rps = static_cast<double>(N * oscillatorFrequency) / numberOfSlits;

    return rps;
}
double calculateMPS(double RPS, double diameter){
    return (RPS * diameter * PI) / 60.0;
}


void setup() {
    // Initiate pulse sensor pins
    pinMode(interruptPinLeft, INPUT_PULLUP);
    pinMode(interruptPinRight, INPUT_PULLUP);

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
    pinMode(lightRightPin, INPUT);
    pinMode(lightLeftPin, INPUT);

    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(interruptPinRight), trigRightWheel, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptPinLeft), trigLeftWheel, CHANGE);
}


void loop() {
    followMode();
    delay(50);
}

void followMode() {
    // Calculate PID to adjust motor speeds based on light sensor readings
    // calculate_pid();

    // Calculate and print RPS for the left wheel
    double leftRPS = calculateRPS(leftPulseCount);
    Serial.print("Left wheel RPS: ");
    Serial.println(leftRPS);

    // Reset the pulse count after calculating RPS
    leftPulseCount = 0;

    // Optionally, you can do the same for the right wheel if needed
    double rightRPS = calculateRPS(rightPulseCount);
    Serial.print("Right wheel RPS: ");
    Serial.println(rightRPS);
    rightPulseCount = 0;

    // Print the speed of the car in meters per second
    Serial.print("Speed: ");
    Serial.println(calculateMPS(rightRPS, 6.5));
}