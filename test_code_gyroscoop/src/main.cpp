#include "mpu6050.hpp"
#include <QMC5883LCompass.h>




float alpha;
float previousOutput;
Mpu6050 mpu;
QMC5883LCompass compass;

float filter(float input) {
    // Apply high-pass filter
    float output = alpha * (previousOutput + input);

    // Update previous output for next iteration
    previousOutput = output;

    return input - output;
}

// void setup() {
//   Serial.begin(115200);
//   compass.init();
  
//   Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
//   Serial.println("Calibration will begin in 5 seconds.");
//   delay(5000);

//   Serial.println("CALIBRATING. Keep moving your sensor...");
//   compass.calibrate();

//   Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
//   Serial.println();
//   Serial.print("compass.setCalibrationOffsets(");
//   Serial.print(compass.getCalibrationOffset(0));
//   Serial.print(", ");
//   Serial.print(compass.getCalibrationOffset(1));
//   Serial.print(", ");
//   Serial.print(compass.getCalibrationOffset(2));
//   Serial.println(");");
//   Serial.print("compass.setCalibrationScales(");
//   Serial.print(compass.getCalibrationScale(0));
//   Serial.print(", ");
//   Serial.print(compass.getCalibrationScale(1));
//   Serial.print(", ");
//   Serial.print(compass.getCalibrationScale(2));
//   Serial.println(");");
// }

// void loop() {
//   delay(1000);
// }


float yawOffset = 0; // Initial yaw offset

void setup() {
    mpu.gyroscoopSetup();
    // compass.init();
    // compass.setCalibrationOffsets(1096.00, 781.00, 532.00);
    // compass.setCalibrationScales(0.73, 1.02, 1.52);
    // compass.setSmoothing(10, true);
}

void loop() {
    // Get gyroscope angles
    float roll = mpu.getAngle(1);
    float pitch = mpu.getAngle(2);
    float yaw = mpu.getAngle(3);

    // // Get magnetometer readings
    // compass.read();
    // int x = compass.getX();
    // int y = compass.getY();

    // // Calculate yaw angle from magnetometer
    // float yaw_mag = atan2(y, x) * RAD_TO_DEG;

    // // Correct yaw angle using magnetometer readings (yaw correction)
    // float yaw_gyro = yawOffset + roll;

    // // Combine gyro and magnetometer yaw angles using complementary filter
    // // Alpha is a blending factor between gyro and mag readings (0 to 1)
    // float alpha = 0.98; // You can adjust this value based on your requirements
    // float yaw = alpha * yaw_gyro + (1 - alpha) * yaw_mag;

    // Output yaw angle
    Serial.print("roll: " + String(filter(roll), 0) + " pitch: " + String(filter(pitch), 0));
    Serial.print(" yaw: " + String(filter(yaw), 0));
    Serial.println();

    // // Update yaw offset to account for gyro drift
    // yawOffset = yaw;

    delay(1); // Delay for stability
}