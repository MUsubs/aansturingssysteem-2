#ifndef _HMC5883_HPP
#define _HMC5883_HPP

#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

class Hmc5883{
private:
    const int hmc_addr1 = 0x1E;
    int x,y,z;
    int xmin,xmax,ymin,ymax,zmin,zmax;


public:
    Hmc5883();
    void magneetSetup();
    float getAngle();
	
};

#endif