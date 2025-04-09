// #include <Arduino.h>
#include "parameters.h"

class Calculations{
    public:

        Calculations(String hello) {
            hello = hello;
        }
    
        float ArcWheelSpeed(boolean outer, int radius, int wheelDistance, int wantedSpeed){
        return outer ? (wantedSpeed * (radius) + (wheelDistance / 2)): (wantedSpeed * (radius) - (wheelDistance / 2));
        }

        float distanceToCar(int readLeft, int readRight){
        int minste = min(readLeft,readRight);
        return (0.0927611617 * minste - 0.7325530993); //bad function, find one that actually works
        }
        double distanceDriven(int countLeft, int countRight) {
        double avgCount = (countLeft + countRight) / 2.0;
        return (avgCount / NUM_SLITS) * WHEEL_DIAMETER * PI;
        }
};