#include <Arduino.h>
#ifndef MOTOR_H
#define MOTOR_H

class Motor {
    private:
        int ENA;
        int IN1;
        int IN2;
        String name;

    public:

        //Constructor
        Motor(String motorName, int ena, int in1, int in2) {
            ENA = ena;
            IN1 = in1;
            IN2 = in2;
            name = motorName;
        }

    void run(int speed, boolean drive, boolean direction){

    if(!drive){
        digitalWrite(ENA, LOW);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        return;
    }
    switch(direction){
        case true:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, speed);
            break;
        case false:
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, speed);
            break;      
    }
}

    void stop(){
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(ENA, LOW);
    }


};

#endif // MOTOR_H