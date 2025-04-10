#include <Arduino.h>
#include "parameters.h"
#include "pid.h"
#include "motor.h"
#include "calculations.h"

// -------------------- PIN DEFINITIONS --------------------
const int interruptPinLeft = 2;
const int interruptPinRight = 3;
const int IN1 = 8;
const int IN2 = 7;
const int ENA = 9;
const int IN3 = 5;
const int IN4 = 4;
const int ENB = 10;
const int lightRightPin = A1;
const int lightLeftPin = A0;

// -------------------- GLOBAL VARIABLES --------------------
volatile long rightPulseCount = 0;
volatile long leftPulseCount = 0;
long lastRightPulseCount = 0;
long lastLeftPulseCount = 0;
unsigned long previousTime = 0;
unsigned long lastTime = 0;
float leftWheelSpeedArc = 0;
float rightWheelSpeedArc = 0;
float totalDistanceDriven = 0.0;
float targetArcDistance = 2 * PI * ARC_TURN_RADIUS;
int baseSpeed = 0;
int arcWantedSpeedBits = BASE_PWM_ARC;
int startError = 0;
int startSum;

// -------------------- CLASS INSTANTIATIONS --------------------
PID anglePid("anglePID", ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD, ANGLE_PID_MAX_INTEGRAL);
PID speedPid("speedPID", SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_MAX_INTEGRAL);
Motor leftMotor("left", ENA, IN1, IN2);
Motor rightMotor("right", ENB, IN3, IN4);
Calculations calculate("");

// -------------------- INTERRUPT HANDLERS --------------------
void trigLeftWheel() {
    leftPulseCount++;
}

void trigRightWheel() {
    rightPulseCount++;
}

// -------------------- SETUP FUNCTION --------------------
void setup() {
    Serial.begin(115200);
    pinMode(interruptPinLeft, INPUT_PULLUP);
    pinMode(interruptPinRight, INPUT_PULLUP);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(lightRightPin, INPUT);
    pinMode(lightLeftPin, INPUT);

    startError = (analogRead(lightLeftPin) - analogRead(lightRightPin)) / 2;
    startSum = (analogRead(lightLeftPin) + analogRead(lightRightPin));

    attachInterrupt(digitalPinToInterrupt(interruptPinRight), trigRightWheel, FALLING);
    attachInterrupt(digitalPinToInterrupt(interruptPinLeft), trigLeftWheel, FALLING);

    if (MODE == "LINE") {
        leftMotor.run(WANTED_SPEED_BITS_LINE, true, false);
        rightMotor.run(WANTED_SPEED_BITS_LINE, true, false);
    } else if (MODE == "ARC") {
        leftWheelSpeedArc = calculate.ArcWheelSpeed(true, ARC_TURN_RADIUS, WHEEL_DISTANCE, ARC_WANTED_SPEED);
        rightWheelSpeedArc = calculate.ArcWheelSpeed(false, ARC_TURN_RADIUS, WHEEL_DISTANCE, ARC_WANTED_SPEED);
        leftMotor.run(BASE_PWM_ARC, true, false);
        rightMotor.run(BASE_PWM_ARC, true, false);
    }
}

// -------------------- LOOP FUNCTION --------------------
void loop() {
    if (MODE == "FOLLOW") {
        followMode();
    } else if (MODE == "LINE") {
        lineMode();
    } else if (MODE == "ARC") {
        arcMode(targetArcDistance);
    } else if (MODE == "SPEED_TEST") {
        speedTestMode();
    } else {
        Serial.println("INVALID MODE SELECTION, FIX IN parameters.h");
    }
}

// -------------------- FOLLOW MODE --------------------
void followMode() {
    unsigned long now = millis();

    int readLightLeft = analogRead(lightLeftPin) - startError;
    int readLightRight = analogRead(lightRightPin) + startError;
    int currentSum = readLightLeft + readLightRight;

    float distance = calculate.distanceToCar(readLightLeft, readLightRight, startSum);

    float angleError = readLightLeft - readLightRight;
    float angleCorrection = anglePid.calculatePidOutput(angleError, lastTime);

    float distanceError = FOLLOW_TARGET_DISTANCE - distance;
    float distanceCorrection = speedPid.calculatePidOutput(distanceError, lastTime);
    
    int leftMotorSpeed = constrain(distanceCorrection + angleCorrection * CONTROL_SENSITIVITY, -255, 255);
    int rightMotorSpeed = constrain(distanceCorrection - angleCorrection * CONTROL_SENSITIVITY, -255, 255);
    leftMotor.run(abs(leftMotorSpeed), leftMotorSpeed >= 0, leftMotorSpeed < 0);
    rightMotor.run(abs(rightMotorSpeed), rightMotorSpeed >= 0, rightMotorSpeed < 0);
    lastTime = now;
}

// -------------------- LINE MODE --------------------
void lineMode() {
    unsigned long now = millis();
    if (now - lastTime >= CONTROL_INTERVAL) {
        float dt = (now - lastTime) / 1000.0;
        noInterrupts();
        long leftPulses = leftPulseCount - lastLeftPulseCount;
        long rightPulses = rightPulseCount - lastRightPulseCount;
        lastLeftPulseCount = leftPulseCount;
        lastRightPulseCount = rightPulseCount;
        interrupts();
        float leftRPS = (float)leftPulses / (NUM_SLITS * dt);
        float rightRPS = (float)rightPulses / (NUM_SLITS * dt);

        float speedError = leftRPS - rightRPS;
        float speedCorrection = speedPid.calculatePidOutput(speedError, lastTime);
        int leftMotorSpeed = constrain(WANTED_SPEED_BITS_LINE - speedCorrection * CONTROL_SENSITIVITY, 0, 255);
        int rightMotorSpeed = constrain(WANTED_SPEED_BITS_LINE + speedCorrection * CONTROL_SENSITIVITY, 0, 255);
        leftMotor.run(leftMotorSpeed, true, false);
        rightMotor.run(rightMotorSpeed, true, false);
        lastTime = now;
    }
}

// -------------------- ARC MODE --------------------
void arcMode(float targetDistance) {
    unsigned long now = millis();
    noInterrupts();
    long localLeftPulseCount = leftPulseCount - lastLeftPulseCount;
    long localRightPulseCount = rightPulseCount - lastRightPulseCount;
    lastLeftPulseCount = leftPulseCount;
    lastRightPulseCount = rightPulseCount;
    interrupts();

    float leftSpeed = calculate.speedFromPulses(localLeftPulseCount, WHEEL_DIAMETER, CONTROL_INTERVAL);
    float rightSpeed = calculate.speedFromPulses(localRightPulseCount, WHEEL_DIAMETER, CONTROL_INTERVAL);
    float leftDistance = calculate.distanceDriven(lastLeftPulseCount, lastRightPulseCount);

    totalDistanceDriven += leftDistance;
    if (totalDistanceDriven >= targetDistance) {
        leftMotor.run(0, true, false);
        rightMotor.run(0, true, false);
        Serial.println("Arc completed, stopping motors.");
    } else {
        float errorLeft = leftWheelSpeedArc - leftSpeed;
        float errorRight = rightWheelSpeedArc - rightSpeed;
        float pidLeft = speedPid.calculatePidOutput(errorLeft, lastTime);
        float pidRight = speedPid.calculatePidOutput(errorRight, lastTime);
        int leftMotorSpeed = constrain(BASE_PWM_ARC + pidLeft * CONTROL_SENSITIVITY_ARC, 0, 255);
        int rightMotorSpeed = constrain(BASE_PWM_ARC + pidRight * CONTROL_SENSITIVITY_ARC, 0, 255);
        leftMotor.run(leftMotorSpeed, true, false);
        rightMotor.run(rightMotorSpeed, true, false);
        lastTime = now;
    }
}

// -------------------- SPEED TEST MODE --------------------
void speedTestMode() {
    unsigned long now = millis();
    if (now - lastTime >= CONTROL_INTERVAL) {
        leftMotor.run(50, true, false);
        rightMotor.run(50, true, false);
        noInterrupts();
        long localLeftPulseCount = leftPulseCount - lastLeftPulseCount;
        long localRightPulseCount = rightPulseCount - lastRightPulseCount;
        lastLeftPulseCount = leftPulseCount;
        lastRightPulseCount = rightPulseCount;
        interrupts();
        calculate.speedFromPulsesRPS(localLeftPulseCount, CONTROL_INTERVAL);
        calculate.speedFromPulsesRPS(localRightPulseCount, CONTROL_INTERVAL);
        lastTime = now;
    }
}