#include "hmc5883.hpp"

Hmc5883::Hmc5883() {
}

// Setup for the HMC5883 magnetisch felt meter
void Hmc5883::magneetSetup() {
    Wire.begin();                                      
    Wire.beginTransmission(hmc_addr1);              
    Wire.write(0x02);       
    Wire.write(0);
    Wire.endTransmission(true);                        
    Serial.begin(115200);
}

float Hmc5883::getAngle() {
  xmin=0; xmax=0; ymax=0; ymin = 0; zmin=0;zmax=0;
  Wire.beginTransmission(hmc_addr1);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(hmc_addr1, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  return z;
  
}