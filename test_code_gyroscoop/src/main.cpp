#include<Wire.h>
#include <Arduino.h>

const int MPU_addr1 = 0x68;
float xa, ya, za, roll, pitch;

void setup() {

  Wire.begin();                                      //begin the wire communication
  Wire.beginTransmission(MPU_addr1);                 //begin, send the slave adress (in this case 68)
  Wire.write(0x6B);                                  //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);                        //end the transmission
  Serial.begin(115200);
}

void loop() {

  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);  //send starting register address
  Wire.endTransmission(false); //restart for read
  Wire.requestFrom(MPU_addr1, 6, true); //get six bytes accelerometer data

  xa = Wire.read() << 8 | Wire.read();
  ya = Wire.read() << 8 | Wire.read();
  za = Wire.read() << 8 | Wire.read();
// formula from https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  roll = atan2(ya , za) * 180.0 / PI;
  pitch = atan2(-xa , sqrt(ya * ya + za * za)) * 180.0 / PI; //account for roll already applied

  roll += 1;
  pitch += 1.5;

  Serial.println("roll = " + String(roll,0) + ", pitch = " + String(pitch,0));
  delay(0.5);
}