#include <Arduino.h>

class Motor {
    private:
        int ENA;
        int IN1;
        int IN2;
        String name;

    public:

        //Constructor
        Motor(String motorName, int ena, int in1, int in2) {
            this->ENA = ena;
            this->IN1 = in1;
            this->IN2 = in2;
            this->name = motorName;
        }

    void run(int speed, boolean drive, boolean direction){

    if(!drive){
        digitalWrite(ENA, LOW);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        Serial.print(name);
        Serial.println(" Motor stopped running!");
        return;
    }
    switch(direction){
        case true:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, speed);
            Serial.print(name);
            Serial.println(" Motor running clockwise!");
            break;
        case false:
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, speed);
            Serial.print(name);
            Serial.println(" Motor running counter-clockwise!");
            break;      
    }
}


};

