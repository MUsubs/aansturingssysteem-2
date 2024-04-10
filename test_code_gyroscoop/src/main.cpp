#include "mpu6050.hpp"

Mpu6050 mpu;

void setup(){
    mpu.gyroscoopSetup();
}

void loop() {
    Serial.println("roll: " + String(mpu.getAngle(1),0) + " pitch: " + String(mpu.getAngle(2),0));
    delay(1); //Delay mag niet super laag anders kan hij niet meten en loopt hij vast.
}