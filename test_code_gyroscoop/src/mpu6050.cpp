#include "mpu6050.hpp"

Mpu6050::Mpu6050() {
}

void Mpu6050::gyroscoopSetup() {
    Wire.begin();                                      //begin the wire communication
    Wire.beginTransmission(mpu_addr1);                 //begin, send the slave adress (in this case 68)
    Wire.write(0x6B);                                  //make the reset (place a 0 into the 6B register)
    Wire.write(0);
    Wire.endTransmission(true);                        //end the transmission
    Serial.begin(115200);
}

float Mpu6050::getAngle(int as) {

  Wire.beginTransmission(mpu_addr1);
  Wire.write(0x3B);  //send starting register address
  Wire.endTransmission(false); //restart for read
  Wire.requestFrom(mpu_addr1, 6, true); //get six bytes accelerometer data

  x_as = Wire.read() << 8 | Wire.read();
  y_as = Wire.read() << 8 | Wire.read();
  z_as = Wire.read() << 8 | Wire.read();
// formula from https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  roll = atan2(y_as , z_as) * 180.0 / PI;
  pitch = atan2(-x_as , sqrt(y_as * y_as + z_as * z_as)) * 180.0 / PI; //account for roll already applied

  roll += 1;
  pitch += 1.5;

//  Serial.println("roll = " + String(roll,0) + ", pitch = " + String(pitch,0));
  if (as == 1){
    return roll;
  }else{
    return pitch;
  }
}