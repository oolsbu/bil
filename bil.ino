#include "parameters.h"
#include <Arduino.h>
#include "pid.h"
#include "motor.h"
#include "calculations.h"

// Pulse counter pins
int interruptPinLeft = 2;
int interruptPinRight = 3;

// Defining pulse counters
volatile long rightPulseCount = 0;
volatile long leftPulseCount = 0;
long lastRightPulseCount = 0;
long lastLeftPulseCount = 0;


// Defining statistics
int rightWheelRotations = 0;
int leftWheelRotations = 0;
int rightWheelSpeed = 0;
int leftWheelSpeed = 0;
unsigned long previousTime = 0;

// Motor A
int IN1 = 8;
int IN2 = 7;
int ENA = 9;

// Motor B
int IN3 = 5;
int IN4 = 4;
int ENB = 10;

//base speed
int startSpeed = STARTSPEED_VALUE;
int baseSpeed = startSpeed;
int actualSpeed = 0;

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

//class instantiation
PID anglePid("anglePID", KP_VERDI, KI_VERDI, KD_VERDI);
// Skal vi adde egne kp, ki og kd verdier for både retning og fart?
PID SpeedPid("speedPID", KP_VERDI, KI_VERDI, KD_VERDI);

Motor leftMotor("left", ENA, IN1, IN2);
Motor rightMotor("right", ENB, IN3, IN4);

Calculations calculate("");

void setup() {
    Serial.begin(9600);

    // Initiate pulse sensor pins
    pinMode(interruptPinLeft, INPUT_PULLUP);
    pinMode(interruptPinRight, INPUT_PULLUP);

    // Initiate motor A pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    // Initiate motor B pins
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Initiate light resistor pins
    pinMode(lightRightPin, INPUT);
    pinMode(lightLeftPin, INPUT);

    attachInterrupt(digitalPinToInterrupt(interruptPinRight), trigRightWheel, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptPinLeft), trigLeftWheel, CHANGE);
}


void loop() {
    followMode();
}

void followMode() {
    // Temporarily disable interrupts to safely access pulse counters
    noInterrupts();
    long localLeftPulseCount = leftPulseCount;
    long localRightPulseCount = rightPulseCount;
    interrupts();

    readLightRight = analogRead(lightRightPin);
    readLightLeft = analogRead(lightLeftPin);
    int actualSpeedLeft = calculate.MPS(localLeftPulseCount, lastLeftPulseCount, WHEEL_DIAMETER);
    int actualSpeedRight = calculate.MPS(localRightPulseCount, lastRightPulseCount, WHEEL_DIAMETER);

    // Update last pulse counts
    lastLeftPulseCount = localLeftPulseCount;
    lastRightPulseCount = localRightPulseCount;

    actualSpeed = (actualSpeedLeft + actualSpeedRight) / 2;

    // fix for distance not speed
    baseSpeed = baseSpeed + SpeedPid.calculatePidOutput(baseSpeed - actualSpeed, previousTime);
    baseSpeed = constrain(baseSpeed, 0, 255);
    float pidOutput = anglePid.calculatePidOutput(readLightLeft - readLightRight, previousTime);

    previousTime = millis();

    int leftMotorSpeed = constrain(baseSpeed + pidOutput, 0, 255);
    int rightMotorSpeed = constrain(baseSpeed - pidOutput, 0, 255);

    // Run motors with adjusted speeds
    leftMotor.run(leftMotorSpeed, true, true);
    rightMotor.run(rightMotorSpeed, true, true);
}