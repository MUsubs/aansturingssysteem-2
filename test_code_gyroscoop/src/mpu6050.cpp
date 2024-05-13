#include "mpu6050.hpp"

Mpu6050::Mpu6050() {
}

// Setup for the MPU6050 gyroscope
void Mpu6050::gyroscoopSetup() {
    Wire.begin();                                      
    Wire.beginTransmission(mpu_addr1);                 
    Wire.write(0x6B);                                  
    Wire.write(0);
    Wire.endTransmission(true);                        
    Serial.begin(115200);
}

// Get the current angle detected by the gyroscope
float Mpu6050::getAngle(int as) {

  Wire.beginTransmission(mpu_addr1);
  Wire.write(0x3B);  
  Wire.endTransmission(false); 
  Wire.requestFrom(mpu_addr1, 6, true); 

  x_as = Wire.read() << 8 | Wire.read();
  y_as = Wire.read() << 8 | Wire.read();
  z_as = Wire.read() << 8 | Wire.read();
// formula from https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  roll = atan2(y_as , z_as) * 180.0 / PI;
  pitch = atan2(-x_as , sqrt(y_as * y_as + z_as * z_as)) * 180.0 / PI;
  roll += 1;
  pitch += 2;

//  Serial.println("roll = " + String(roll,0) + ", pitch = " + String(pitch,0));
  if (as == 1){
    return roll;
  }else{
    return pitch;
  }
}