#include <Arduino.h>
#include "parameters.h"
#include "pid.h"
#include "motor.h"
#include "calculations.h"

// -------------------- HELPER FUNCTIONS --------------------
int calculateInitialPWM(float rpm) {
    return 80; // Ensure PWM is within valid range
}
// -------------------- PIN DEFINITIONS --------------------
// Pulse counter pins
const int interruptPinLeft = 2;
const int interruptPinRight = 3;

// Motor A
const int IN1 = 8;
const int IN2 = 7;
const int ENA = 9;

// Motor B
const int IN3 = 5;
const int IN4 = 4;
const int ENB = 10;

// Photoresistors
const int lightRightPin = A1;
const int lightLeftPin = A0;

// -------------------- GLOBAL VARIABLES --------------------
// Pulse counters
volatile long rightPulseCount = 0;
volatile long leftPulseCount = 0;
long lastRightPulseCount = 0;
long lastLeftPulseCount = 0;

// Timing
unsigned long previousTime = 0;
unsigned long lastTime = 0;

// Motor speeds
float leftWheelSpeedArc = 0;
float rightWheelSpeedArc = 0;
int setSpeed =  calculateInitialPWM(0.5 * 60 / (PI * WHEEL_DIAMETER));
int baseSpeed = 0;
int arcWantedSpeedBits = calculateInitialPWM(ARC_WANTED_SPEED * 60 / (PI * WHEEL_DIAMETER));

// Light sensor readings
int readLightRight = 0;
int readLightLeft = 0;
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

    // Initialize pulse sensor pins
    pinMode(interruptPinLeft, INPUT_PULLUP);
    pinMode(interruptPinRight, INPUT_PULLUP);

    // Initialize motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Initialize light resistor pins
    pinMode(lightRightPin, INPUT);
    pinMode(lightLeftPin, INPUT);

    // Calculate initial light sensor error
    startError = (analogRead(lightLeftPin) - analogRead(lightRightPin)) / 2;
    startSum = (analogRead(lightLeftPin) + analogRead(lightRightPin));

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(interruptPinRight), trigRightWheel, FALLING);
    attachInterrupt(digitalPinToInterrupt(interruptPinLeft), trigLeftWheel, FALLING);

    // Initialize mode-specific parameters
    if (MODE == "LINE") {
        int initialPWM = WANTED_SPEED_BITS_LINE;
        leftMotor.run(initialPWM, true, false);
        rightMotor.run(initialPWM, true, false);
    } else if (MODE == "ARC") {
        leftWheelSpeedArc = calculate.ArcWheelSpeed(true, ARC_TURN_RADIUS, WHEEL_DISTANCE, ARC_WANTED_SPEED);
        rightWheelSpeedArc = calculate.ArcWheelSpeed(false, ARC_TURN_RADIUS, WHEEL_DISTANCE, ARC_WANTED_SPEED);

        // Serial.print("left init speed: ");
        // Serial.print(leftWheelSpeedArc);
        // Serial.print(" right init speed: ");
        // Serial.println(rightWheelSpeedArc);
        
        int leftInitialPWM = calculateInitialPWM(leftWheelSpeedArc * 60 / (PI * WHEEL_DIAMETER)); // Convert m/s to RPM
        int rightInitialPWM = calculateInitialPWM(rightWheelSpeedArc * 60 / (PI * WHEEL_DIAMETER)); // Convert m/s to RPM
        
        // Serial.print("left init speed pwm: ");
        // Serial.print(leftInitialPWM);
        // Serial.print(" right init speed pwm: ");
        // Serial.println(rightInitialPWM);
        
        leftMotor.run(leftInitialPWM, true, false);
        rightMotor.run(rightInitialPWM, true, false);
    }
}

// -------------------- LOOP FUNCTION --------------------
void loop() {
    if (MODE == "FOLLOW") {
        followMode();
    } else if (MODE == "LINE") {
        lineMode(LINE_TARGET_SPEED);
    } else if (MODE == "ARC") {
        arcMode(ARC_TURN_RADIUS * PI);
    } else if (MODE == "SPEED_TEST") {
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
     else {
        // Serial.println("INVALID MODE SELECTION, FIX IN parameters.h");
}
}

// -------------------- FOLLOW MODE --------------------
void followMode() {
    // Read light sensor values and adjust for starting error
    readLightLeft = analogRead(lightLeftPin) - startError;
    readLightRight = analogRead(lightRightPin) + startError;

    if (DEBUG_MODE) {
        Serial.print("Start sum: ");
        Serial.print(startSum);
        Serial.print(" Start Error: ");
        Serial.print(startError);
        Serial.print(" Left: ");
        Serial.print(readLightLeft);
        Serial.print(" Right: ");
        Serial.print(readLightRight);
        Serial.print(" sum: ");
        Serial.print(readLightLeft + readLightRight);
    }

    // Calculate distance to the car in front
    
    float distance = calculate.distanceToCar(readLightLeft, readLightRight, startSum);
    Serial.print(" distance: ");
    Serial.print(distance);

    // Use distance in a PID controller to adjust baseSpeed
    float distanceError = FOLLOW_TARGET_DISTANCE - distance;
    int baseSpeed = constrain(setSpeed * (distance - FOLLOW_TARGET_DISTANCE), 0, 255);
    Serial.print("baseSpeed: ");
    Serial.println(baseSpeed);

    // Use angle PID controller to adjust for light difference
    float pidOutput = anglePid.calculatePidOutput(readLightLeft - readLightRight, previousTime);

    previousTime = millis();

    // Calculate motor speeds
    int leftMotorSpeed = constrain(baseSpeed + pidOutput * CONTROL_SENSITIVITY, 0, 255);
    int rightMotorSpeed = constrain(baseSpeed - pidOutput * CONTROL_SENSITIVITY, 0, 255);

    // Run motors with adjusted speeds
    leftMotor.run(leftMotorSpeed, true, false);
    rightMotor.run(rightMotorSpeed, true, false);
}

// -------------------- LINE MODE --------------------
void lineMode(float targetRPS) {
    unsigned long now = millis();
    if (now - lastTime >= CONTROL_INTERVAL) {
        // Safely access pulse counters
        noInterrupts();
        long localLeftPulseCount = leftPulseCount - lastLeftPulseCount;
        long localRightPulseCount = rightPulseCount - lastRightPulseCount;
        lastLeftPulseCount = leftPulseCount;
        lastRightPulseCount = rightPulseCount;
        interrupts();

        int pulseDiff = (lastLeftPulseCount - lastRightPulseCount);

        // Serial.print("left: ");
        // Serial.print(lastLeftPulseCount);
        // Serial.print( "right: ");
        // Serial.print(lastRightPulseCount);
        // Serial.print("diff: ");
        // Serial.println(pulseDiff);

        // Use PID controllers to adjust motor speeds
        float pid = speedPid.calculatePidOutput(pulseDiff, lastTime);

        int leftMotorSpeed = constrain(WANTED_SPEED_BITS_LINE - pid * CONTROL_SENSITIVITY, 0, 255);
        int rightMotorSpeed = constrain(WANTED_SPEED_BITS_LINE + pid * CONTROL_SENSITIVITY, 0, 255);

        // Run motors with adjusted speeds
        leftMotor.run(leftMotorSpeed, true, false);
        rightMotor.run(rightMotorSpeed, true, false);

        lastTime = now;
    }
}

// -------------------- ARC MODE --------------------
void arcMode(float targetDistance) {
    unsigned long now = millis();
    if (now - lastTime >= CONTROL_INTERVAL) {
        // Safely access pulse counters
        noInterrupts();
        long localLeftPulseCount = leftPulseCount - lastLeftPulseCount;
        long localRightPulseCount = rightPulseCount - lastRightPulseCount;
        lastLeftPulseCount = leftPulseCount;
        lastRightPulseCount = rightPulseCount;
        interrupts();

        // Calculate target speeds for the arc
        leftWheelSpeedArc = calculate.ArcWheelSpeed(true, ARC_TURN_RADIUS, WHEEL_DISTANCE, ARC_WANTED_SPEED);
        rightWheelSpeedArc = calculate.ArcWheelSpeed(false, ARC_TURN_RADIUS, WHEEL_DISTANCE, ARC_WANTED_SPEED);

        // Calculate actual speeds from pulse counts
        float actualSpeedLeft = calculate.speedFromPulses(localLeftPulseCount, WHEEL_DIAMETER, CONTROL_INTERVAL);
        float actualSpeedRight = calculate.speedFromPulses(localRightPulseCount, WHEEL_DIAMETER, CONTROL_INTERVAL);

        // Calculate speed errors
        float errorLeft = leftWheelSpeedArc - actualSpeedLeft;
        float errorRight = rightWheelSpeedArc - actualSpeedRight;

        // Use PID controllers to adjust motor speeds
        float pidLeft = speedPid.calculatePidOutput(errorLeft, lastTime);
        float pidRight = speedPid.calculatePidOutput(errorRight, lastTime);

        int leftMotorSpeed = constrain(calculateInitialPWM(leftWheelSpeedArc * 60 / (PI * WHEEL_DIAMETER)) + pidLeft * CONTROL_SENSITIVITY_ARC, 0, 255);
        int rightMotorSpeed = constrain(calculateInitialPWM(rightWheelSpeedArc * 60 / (PI * WHEEL_DIAMETER)) + pidRight * CONTROL_SENSITIVITY_ARC, 0, 255);

        // Debugging output
        if (DEBUG_MODE) {
            // Serial.print("Target Left Speed: ");
            // Serial.print(leftWheelSpeedArc);
            // Serial.print(" Actual Left Speed: ");
            // Serial.print(actualSpeedLeft);
            // Serial.print(" Target Right Speed: ");
            // Serial.print(rightWheelSpeedArc);
            // Serial.print(" Actual Right Speed: ");
            // Serial.println(actualSpeedRight);

            // Serial.print("PID Left: ");
            // Serial.print(pidLeft);
            // Serial.print(" PID Right: ");
            // Serial.println(pidRight);

            // Serial.print("Motor Left Speed: ");
            // Serial.print(leftMotorSpeed);
            // Serial.print(" Motor Right Speed: ");
            // Serial.println(rightMotorSpeed);
        }

        // Run motors with adjusted speeds
        leftMotor.run(leftMotorSpeed, true, false);
        rightMotor.run(rightMotorSpeed, true, false);

        lastTime = now;
    }
}