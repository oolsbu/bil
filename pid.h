// #include <arduino.h>
#include "motor.h"
#include "parameters.h"

class PID {
    private: 
        float Kp = KP_VERDI;
        float Ki = KI_VERDI;
        float Kd = KD_VERDI;
        String name;

        float error = 0;
        float previousError = 0;
        float integral = 0;
        float derivative = 0;

    public: 
        PID(String name){
            name = name;
            
    }
    float calculatePidOutput(float error, unsigned long previousTime) {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - previousTime) / 1000.0;

        // Calculate integral (accumulated error over time)
        integral += error * deltaTime;
        integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

        // Calculate derivative (rate of change of error)
        derivative = (error - previousError) / deltaTime;

        // Calculate PID output
        float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
        if(DEBUG == 1){
            Serial.print("error: ");
            Serial.print(error);
            Serial.print(" integral: ");
            Serial.print(integral);
            Serial.print(" derivative: ");
            Serial.print(derivative);
            Serial.print(" pidOutput: ");
            Serial.println(pidOutput);
        }

        // Update previous error
        previousError = error; 

        return pidOutput;
    }
};
