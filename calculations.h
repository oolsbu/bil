#include <Arduino.h>
#include "parameters.h"

class Calculations{
    public:

        Calculations(String hello) {
            this->hello = hello;
        }
    
        float ArcWheelSpeed(boolean outer, int radius, int wheelDistance, int wantedSpeed){
        return outer ? (wantedSpeed * (radius) + (wheelDistance / 2)): (wantedSpeed * (radius) - (wheelDistance / 2));
        }
        double MPS(int counter, int lastCount, double diameter){
            const int numberOfSlits = NUM_SLITS;
            const int oscillatorFrequency = FREQUENCY;

            int N = counter - lastCount;

            double rps = static_cast<double>(N * oscillatorFrequency) / numberOfSlits;

            return (rps * diameter * PI);
        }

        float Distance(int readLeft, int readRight){
        int minste = min(readLeft,readRight);
        return (0.0927611617 * minste - 0.7325530993); //bad function, find one that actually works
        }
};

//TODO
//Find out about the oscillatorFrequency thingy