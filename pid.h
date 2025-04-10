// #include <arduino.h>
#include "motor.h"
#include "parameters.h"

class PID {
    private: 
        float Kp;
        float Ki;
        float Kd;
        float maxIntegral;
        String name;

        float error = 0;
        float previousError = 0;
        float integral = 0;
        float derivative = 0;

    public: 
        PID(String pidName, float kp, float ki, float kd, float maxInt) {
            name = pidName;
            Kp = kp;
            Ki = ki;
            Kd = kd;
            maxIntegral = maxInt;
        }

        float calculatePidOutput(float error, unsigned long previousTime) {
            unsigned long currentTime = millis();
            float deltaTime = (currentTime - previousTime) / 1000.0;

            // Calculate integral (accumulated error over time)
            integral += error * deltaTime;
            integral = constrain(integral, -maxIntegral, maxIntegral);

            // Calculate derivative (rate of change of error)
            derivative = (error - previousError) / deltaTime;

            // Calculate PID output
            float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
            if (DEBUG_MODE == 1) {
                // Serial.print("PID [");
                // Serial.print(name);
                // Serial.print("] error: ");
                // Serial.print(error);
                // Serial.print(" integral: ");
                // Serial.print(integral);
                // Serial.print(" derivative: ");
                // Serial.print(derivative);
                // Serial.print(" pidOutput: ");
                // Serial.println(pidOutput);
            }

            // Update previous error
            previousError = error; 

            return pidOutput;
        }
};
