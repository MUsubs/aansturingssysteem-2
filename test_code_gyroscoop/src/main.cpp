#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>
#include <Kalman.h>


MPU6050 mpu(Wire);
unsigned long timer = 0;
Servo my_servo;
Kalman kalmanFilter;

Kalman kalmanFilterX;
Kalman kalmanFilterY;
Kalman kalmanFilterZ;

float angleX, angleY, angleZ;
unsigned long prevTime;
float dt;



// High-pass filter variabelen
float alpha = 0.8; // Instelbare filterfactor
float previous_x, previous_y, previous_z;
float servo_pos = 90;


// Kalman-filter variabelen
double Q = 1; // Procesruis
double R = 100;   // Meetruis
double x = 0;   // Geschatte staat
double P = 1;   // Geschatte foutcovariantie
double K;       // Kalman-gain

// Functie voor het toepassen van een high-pass filter
float highPassFilter(float current_value, float previous_value) {
    return alpha * (previous_value + current_value);
}


void set(int pos, int val){
    int newer = val;
    int older = pos;
    if ( newer > older ){
        while ( newer > older ) {
            newer = newer - 1;
            my_servo.write(newer);
            delay(50);
        }
    }
    if ( older > newer ) {
        while ( older > newer ) {
            newer = newer + 1;
            my_servo.write(newer);
            delay(50);
        }
    }
}

// // Functie voor het toepassen van het Kalman-filter
// double kalmanFilter(double measurement) {
//     // Voorspelling
//     double x_pred = x;
//     double P_pred = P + Q;

//     // Update
//     K = P_pred / (P_pred + R);
//     x = x_pred + K * (measurement - x_pred);
//     P = (1 - K) * P_pred;

//     return x; // Retourneer de gefilterde waarde
// }

void setup() {
    Serial.begin(115200);
    Wire.begin();
    my_servo.attach(9);
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    while (status != 0) { } // Stop alles als er geen verbinding met MPU6050 is
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);

    mpu.calcOffsets(); // Gyroscoop en accelerometer kalibreren
    Serial.println("Done!\n");



    // Set initial angles (assuming starting from 0 degrees)
    angleX = 0.0f;
    angleY = 0.0f;
    angleZ = 0.0f;

    // it is best to leave this at the default value.
    // you can change it if needed to fine tune the results
    kalmanFilter.setAngle(0.0f);
    kalmanFilter.setQangle(0.001f);
    kalmanFilter.setQbias(0.003f);
    kalmanFilter.setRmeasure(0.03f);
    prevTime = millis();

}

void loop() {
    mpu.update();


    // Haal gyrowaarden op
    float gyro_x = mpu.getAngleX();
    float gyro_y = mpu.getAngleY();
    float gyro_z = mpu.getAngleZ();

    float gyroXRate = mpu.getGyroX(); // degrees per second
    float gyroYRate = mpu.getGyroY(); // degrees per second
    float gyroZRate = mpu.getGyroZ(); // degrees per second

    // Calculate delta time
    unsigned long currentTime = millis();
    dt = (currentTime - prevTime) / 1000.0; // Convert milliseconds to seconds
    prevTime = currentTime

    // Update Kalman filters with gyro rates
    angleX = kalmanFilterX.getAngle(angleX, gyroXRate, dt);
    angleY = kalmanFilterY.getAngle(angleY, gyroYRate, dt);
    angleZ = kalmanFilterZ.getAngle(angleZ, gyroZRate, dt);

    // Output the filtered angles
    Serial.print("Angle X: "); Serial.println(angleX);
    Serial.print("Angle Y: "); Serial.println(angleY);
    Serial.print("Angle Z: "); Serial.println(angleZ);

    // Pas de high-pass filter toe op elke gyrowaarde
    float current_x = highPassFilter(gyro_x, previous_x);
    float current_y = highPassFilter(gyro_y, previous_y);
    float current_z = highPassFilter(gyro_z, previous_z);

//   // Pas het Kalman-filter toe op elke gefilterde gyrowaarde
//   double filtered_x = kalmanFilter(current_x);
//   double filtered_y = kalmanFilter(current_y);
//   double filtered_z = kalmanFilter(current_z);

//    Serial.print("X : ");
//    Serial.print(round(currentX/4));
//    Serial.print("\tY : ");
//    Serial.print(round(currentY/4));
//    Serial.print("\tZ : ");
//    Serial.print(round(currentZ/4));
//    Serial.print("\tPos : ");
//    Serial.println(servo_pos);

    delay(50);

    if (servo_pos < 180 && servo_pos > 0){
        if(round(current_z)/4 > 0){
            set(servo_pos, servo_pos - 1);
            servo_pos = servo_pos - 1;
        }
        else if(round(current_z)/4 < 0){
            set(servo_pos, servo_pos + 1);
            servo_pos = servo_pos + 1;
        }
    }

    // Update vorige hoeken voor de volgende iteratie
    previous_x = current_x;
    previous_y = current_y;
    previous_z = current_z;

    // get filtered angle
    // newAngle is the angle measurement from your sensor,
    // newRate is the rate of change of the angle (from a gyroscope),
    // and dt is the elapsed time since the last measurement.
    float new_angle = 0;
    float new_rate = 0;
    float dt = 0;
    float filteredAngle = kalmanFilter.getAngle(new_angle, new_rate, dt);

    delay(50);

}
