#include "parameters.h"
// #include <Arduino.h>
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
const unsigned long interval = INTERVAL;
const int pulsesPerRev = NUM_SLITS;

float leftWheelSpeedArc = 0;
float rightWheelSpeedArc = 0;


// Defining statistics
int rightWheelRotations = 0;
int leftWheelRotations = 0;
int rightWheelSpeed = 0;
int leftWheelSpeed = 0;
unsigned long previousTime = 0;
static unsigned long lastTime = 0;

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
int baseSpeed = 50;
int actualSpeed = 0;

// photoresistors
int lightRightPin = A1;
int readLightRight = 0;
int lightLeftPin = A0;
int readLightLeft = 0;

//starting error between the light resistors before it follows the car
int startError = 0;

void trigLeftWheel (){
    leftPulseCount++;
}

void trigRightWheel (){
    rightPulseCount++;
}

//class instantiation
PID anglePid("anglePID");
// Skal vi adde egne kp, ki og kd verdier for både retning og fart?
PID SpeedPid("speedPID");

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

    startError = (analogRead(lightLeftPin) - analogRead(lightRightPin)) / 2;
    attachInterrupt(digitalPinToInterrupt(interruptPinRight), trigRightWheel, FALLING);
    attachInterrupt(digitalPinToInterrupt(interruptPinLeft), trigLeftWheel, FALLING);

    if(MODE == "LINE"){
        leftMotor.run(WANTED_SPEED, true, false);
        rightMotor.run(WANTED_SPEED, true, false);
    }
    if(MODE == "ARC"){
        leftWheelSpeedArc = calculate.ArcWheelSpeed(true, TURN_RADIUS, WHEEL_DISTANCE, ARC_WANTED_SPEED);
        rightWheelSpeedArc = calculate.ArcWheelSpeed(false, TURN_RADIUS, WHEEL_DISTANCE, ARC_WANTED_SPEED);
        leftMotor.run(ARC_WANTED_SPEED_BITS, true, false);
        rightMotor.run(ARC_WANTED_SPEED_BITS, true, false);
    }
}


void loop() {
    if(MODE == "FOLLOW"){
        followMode();
    }
    else if (MODE == "LINE"){
        Serial.print("LINE MODE!");
        lineMode(TARGET_DISTANCE);
    }
    else if(MODE == "ARC"){
        Serial.print("ARC MODE!");
        arcMode(TURN_RADIUS * PI);
    }
    else{
        Serial.println("INVALID MODE SELECTION, FIX IN parameters.h");
    }
    // delay(50);
}

void followMode() {
    // Temporarily disable interrupts to safely access pulse counters
    // noInterrupts();
    // long localLeftPulseCount = leftPulseCount;
    // long localRightPulseCount = rightPulseCount;
    // interrupts();

    // int actualSpeedLeft = calculate.MPS(localLeftPulseCount, lastLeftPulseCount, WHEEL_DIAMETER);
    // int actualSpeedRight = calculate.MPS(localRightPulseCount, lastRightPulseCount, WHEEL_DIAMETER);
    
    readLightLeft = analogRead(lightLeftPin) - startError;
    readLightRight = analogRead(lightRightPin) + startError;

    if(DEBUG == 1){
    Serial.print("Serr: ");
    Serial.print(startError);
    Serial.print(" l: ");
    Serial.print(readLightLeft);
    Serial.print("(");
    Serial.print(readLightLeft + startError);
    Serial.print(")");
    Serial.print(" r: ");
    Serial.print(readLightRight);
    Serial.print("(");
    Serial.print(readLightRight - startError);
    Serial.print(")");
}
    // // Update last pulse counts
    // lastLeftPulseCount = localLeftPulseCount;
    // lastRightPulseCount = localRightPulseCount;

    // actualSpeed = (actualSpeedLeft + actualSpeedRight) / 2;

    // fix for distance not speed
    // baseSpeed = baseSpeed + SpeedPid.calculatePidOutput(baseSpeed - actualSpeed, previousTime);
    // baseSpeed = constrain(baseSpeed, 0, 255);
    float pidOutput = anglePid.calculatePidOutput(readLightLeft - readLightRight, previousTime);

    previousTime = millis();

    int leftMotorSpeed = constrain(baseSpeed + pidOutput * SENSITIVITY , 0, 255);
    int rightMotorSpeed = constrain(baseSpeed - pidOutput * SENSITIVITY, 0, 255);

    // Run motors with adjusted speeds
    leftMotor.run(leftMotorSpeed, true, false);
    rightMotor.run(rightMotorSpeed, true, false);
}
void lineMode(int targetDistance) {
    unsigned long now = millis();
    if (now - lastTime >= interval) {
        noInterrupts();
        long fullPulseCountLeft = leftPulseCount;
        long fullPulseCountRight = rightPulseCount;
        long localLeftPulseCount = leftPulseCount - lastLeftPulseCount;
        long localRightPulseCount = rightPulseCount - lastRightPulseCount;
        lastLeftPulseCount = leftPulseCount;
        lastRightPulseCount = rightPulseCount;
        interrupts();

        float rpsL = (localLeftPulseCount / (float)pulsesPerRev) * (1000.0 / interval);
        float rpsR = (localRightPulseCount / (float)pulsesPerRev) * (1000.0 / interval);

        double distanceDriven = calculate.distanceDriven(leftPulseCount, rightPulseCount);

        if (targetDistance > distanceDriven) {
            float actualSpeedLeft = rpsL * WHEEL_DIAMETER * PI;
            float actualSpeedRight = rpsR * WHEEL_DIAMETER * PI;

            float errorLeft = actualSpeedLeft - STRAIGHTLINE_TARGET;
            float errorRight = actualSpeedRight - STRAIGHTLINE_TARGET;

            float pidLeft = SpeedPid.calculatePidOutput(errorLeft, lastTime);
            float pidRight = SpeedPid.calculatePidOutput(errorRight, lastTime);

            int leftMotorSpeed = constrain(WANTED_SPEED - pidLeft * SENSITIVITY * 100, 0, 255);
            int rightMotorSpeed = constrain(WANTED_SPEED - pidRight * SENSITIVITY * 100, 0, 255);

            // Send detailed data over Serial
            Serial.print("dis:"); Serial.print(distanceDriven);
            Serial.print(",rpsL:"); Serial.print(rpsL);
            Serial.print(",rpsR:"); Serial.print(rpsR);
            Serial.print(",speedL:"); Serial.print(actualSpeedLeft);
            Serial.print(",speedR:"); Serial.print(actualSpeedRight);
            Serial.print(",errorL:"); Serial.print(errorLeft);
            Serial.print(",errorR:"); Serial.print(errorRight);
            Serial.print(",pidL:"); Serial.print(pidLeft);
            Serial.print(",pidR:"); Serial.print(pidRight);
            Serial.print(",motorL:"); Serial.print(leftMotorSpeed);
            Serial.print(",motorR:"); Serial.println(rightMotorSpeed);

            // Run motors with adjusted speeds
            leftMotor.run(leftMotorSpeed, true, false);
            rightMotor.run(rightMotorSpeed, true, false);
        } else {
            leftMotor.stop();
            rightMotor.stop();
            Serial.print("Finished, Rotations left: ");
            Serial.print(leftPulseCount);
            Serial.print(", Rotations right: ");
            Serial.println(rightPulseCount);
        }
        lastTime = now;
    }
}

void arcMode(int targetDistance){
    unsigned long now = millis();
    if(now - lastTime >= interval){
        noInterrupts();
        long fullPulseCountLeft = leftPulseCount;
        long fullPulseCountRight = rightPulseCount;
        long localLeftPulseCount = leftPulseCount - lastLeftPulseCount;
        long localRightPulseCount = rightPulseCount - lastRightPulseCount;
        lastLeftPulseCount = leftPulseCount;
        lastRightPulseCount = rightPulseCount;
        interrupts();
        float rpsL = (localLeftPulseCount / (float)pulsesPerRev) * (1000.0 / interval);
        float rpsR = (localRightPulseCount / (float)pulsesPerRev) * (1000.0 / interval);

        double distanceDriven = calculate.distanceDriven(leftPulseCount, rightPulseCount);
        
        if(targetDistance > distanceDriven){
            Serial.print("dis: ");
            Serial.println(distanceDriven);
            float actualSpeedLeft = rpsL * WHEEL_DIAMETER * PI;
            float actualSpeedRight = rpsR * WHEEL_DIAMETER * PI;
            Serial.print("SPL:");
            Serial.print(actualSpeedLeft);
            Serial.print(" SPR:");
            Serial.println(actualSpeedRight);
            Serial.print("left: ");
            float pidLeft = SpeedPid.calculatePidOutput(actualSpeedLeft - leftWheelSpeedArc, lastTime);
            Serial.print("Right: ");
            float pidRight = SpeedPid.calculatePidOutput(actualSpeedRight - rightWheelSpeedArc, lastTime);
            int leftMotorSpeed = constrain(ARC_WANTED_SPEED_BITS - pidLeft * SENSITIVITY * 100 , 0, 255);
            int rightMotorSpeed = constrain(ARC_WANTED_SPEED_BITS - pidRight * SENSITIVITY * 100, 0, 255);

            // Run motors with adjusted speeds
            Serial.print("Running Left: ");
            Serial.print(leftMotorSpeed);
            Serial.print("Running Right: ");
            Serial.println(rightMotorSpeed);

            leftMotor.run(leftMotorSpeed, true, false);
            rightMotor.run(rightMotorSpeed, true, false);
            
        }
        else{
            leftMotor.stop();
            rightMotor.stop();
            Serial.print("Rotations left: ");
            Serial.print(leftPulseCount);
            Serial.print("Rotations right: ");
            Serial.println(rightPulseCount);
        }
        lastTime = now;
    }
}