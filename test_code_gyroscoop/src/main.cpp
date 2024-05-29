#include "Wire.h"
#include <MPU6050_light.h>
#include <PID_v1.h>
#include <VarSpeedServo.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;
VarSpeedServo my_servo;

// High-pass filter variables
float alpha = 0.8; // Adjustable filter factor
float previous_x = 0, previous_y = 0, previous_z = 0;
float servo_pos = 0;
float pos_prev = 0;

// PID variables
double setpoint = 0;
double error = 0;
double errorsum = 0;
double errorprev = 0;
double errordiv = 0;
double Kp = 1; 
double Ki = 0.5;
double Kd = 0.1;
double dt = 0.1;


// High-pass filter function
float highPassFilter(float current_value, float previous_value) {
    return alpha * (previous_value + current_value - alpha * previous_value);
}


void setup() {
    Serial.begin(9600);
    Wire.begin();
    my_servo.attach(9);
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) { } // Stop everything if there's no connection with MPU6050
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(); // Calibrate gyro and accelerometer
    Serial.println("Done!\n");
}

void loop() {
    mpu.update();

    // Get gyro values
    float gyro_x = mpu.getAngleX();
    float gyro_y = mpu.getAngleY();
    float gyro_z = mpu.getAngleZ();

    // Apply high-pass filter to each gyro value
    float current_x = highPassFilter(gyro_x, previous_x);
    float current_y = highPassFilter(gyro_y, previous_y);
    float current_z = highPassFilter(gyro_z, previous_z);

    // Compute PID
    error = setpoint - current_z;
    errorsum += error*dt;
    errordiv = (error - errorprev) / dt;

    servo_pos = (Kp * error + Ki * errorsum + Kd * errordiv);

    my_servo.write(round(servo_pos / 4), 30);

    errorprev = error;

    // Debugging output
    Serial.print("X : ");
    Serial.print(round(current_x / 4));
    Serial.print("\tY : ");
    Serial.print(round(current_y / 4));
    Serial.print("\tZ : ");
    Serial.print(current_z);
    Serial.print("\tPos : ");
    Serial.print(round(servo_pos / 4));
    Serial.print("\tDif : ");
    Serial.println(error);

    // Update previous angles for the next iteration
    previous_x = current_x;
    previous_y = current_y;
    previous_z = current_z;
    pos_prev = servo_pos;

    delay(5);
}